//! True Random Number Generator (TRNG) driver.
use core::fmt::Display;
#[cfg(feature = "rt")]
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use cortex_m::asm;
use embassy_hal_internal::Peri;
#[cfg(feature = "rt")]
use embassy_sync::waitqueue::AtomicWaker;
use mspm0_metapac::trng::regs::Int;
use mspm0_metapac::trng::vals::Cmd::*;
use mspm0_metapac::trng::vals::{self, PwrenKey, Ratio, RstctlKey};
use rand_core::{TryCryptoRng, TryRng};

use crate::peripherals::TRNG;
use crate::sealed;
use crate::sysctl::{LowPowerInstance, WakeGuard};

/// Woken by the TRNG interrupt, so it only exists where there is one to install.
#[cfg(feature = "rt")]
static WAKER: AtomicWaker = AtomicWaker::new();

/// Decimation rate marker types. See [`DecimRate`].
pub trait SecurityMarker: sealed::Sealed {
    type DecimRate: DecimRate;
}

/// Marker type for cryptographic decimation rate. See [`CryptoDecimRate`].
pub struct Crypto;

impl sealed::Sealed for Crypto {}
impl SecurityMarker for Crypto {
    type DecimRate = CryptoDecimRate;
}

/// Marker type for fast decimation rate. See [`FastDecimRate`].
pub struct Fast;

impl sealed::Sealed for Fast {}
impl SecurityMarker for Fast {
    type DecimRate = FastDecimRate;
}

/// The decimation rate settings for the TRNG.
/// Higher decimation rates improve the quality of the random numbers at the cost of speed.
///
/// L-series TRM 13.2.2: It is required to use a decimation rate of at least 4 for cryptographic applications.
///
/// See [`FastDecimRate`] and [`CryptoDecimRate`] for available options.
pub trait DecimRate: sealed::Sealed + Into<vals::DecimRate> + Copy {}

/// Fast decimation rates for non-cryptographic applications.
/// See [`DecimRate`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FastDecimRate {
    Decim1,
    Decim2,
    Decim3,
}

impl sealed::Sealed for FastDecimRate {}
impl Into<vals::DecimRate> for FastDecimRate {
    fn into(self) -> vals::DecimRate {
        match self {
            Self::Decim1 => vals::DecimRate::Decim1,
            Self::Decim2 => vals::DecimRate::Decim2,
            Self::Decim3 => vals::DecimRate::Decim3,
        }
    }
}
impl DecimRate for FastDecimRate {}

/// Cryptographic decimation rates for cryptographic applications.
/// See [`DecimRate`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CryptoDecimRate {
    Decim4,
    Decim5,
    Decim6,
    Decim7,
    Decim8,
}

impl sealed::Sealed for CryptoDecimRate {}
impl Into<vals::DecimRate> for CryptoDecimRate {
    fn into(self) -> vals::DecimRate {
        match self {
            Self::Decim4 => vals::DecimRate::Decim4,
            Self::Decim5 => vals::DecimRate::Decim5,
            Self::Decim6 => vals::DecimRate::Decim6,
            Self::Decim7 => vals::DecimRate::Decim7,
            Self::Decim8 => vals::DecimRate::Decim8,
        }
    }
}
impl DecimRate for CryptoDecimRate {}

/// Represents errors that can arise during initialization with [`Trng::new`] or reading a value from the [`Trng`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// L-series TRM
    /// The digital startup health test is run by application software when powering up the TRNG module.
    /// This built-in self-test verifies that the digital block is functioning properly by running predefined sequences of digital samples through the complete digital block while checking for expected outputs.
    /// The test sequence includes eight tests (indexed 0-7.)
    ///
    /// This failure indicates an irrecoverable fault in the digital block detected during startup.
    DigitalSelfTestFailed(u8),
    /// L-series TRM 13.2.4.2:
    /// The analog startup health test is run by application software when powering up the TRNG module.
    /// This test verifies that the analog block is functioning properly by capturing 4,096 consecutive analog samples and verifying that the samples pass a health test.
    ///
    /// The driver only raises this error after three subsequent failures during initialization.
    AnalogSelfTestFailed,
    /// L-series TRM 13.2.4.3.1:
    /// The repetition count test quickly detects failures that cause the entropy source to remain stuck on a single output value for an extended period of time.
    /// The repetition count test fails if the entropy source outputs the same bit value for 135 consecutive samples.
    ///
    /// <div class="warning"> This error may occur by chance and can be retried up to two additional times after calling <a href="struct.Trng.html#method.fail_reset"><code>Trng::fail_reset</code></a>. </div>
    RepetitionCountTestFailed,
    /// L-series TRM 13.2.4.3.2:
    /// The adaptive proportion test detects failures that cause a disproportionate number of samples to be the same bit value and/or bit pattern over a window of 1024 samples.
    /// The adaptive proportion test fails if any of the conditions in Table 13-1 are violated.
    ///
    /// <div class="warning"> This error may occur by chance and can be retried up to two additional times after calling <a href="struct.Trng.html#method.fail_reset"><code>Trng::fail_reset</code></a>. </div>
    AdaptiveProportionTestFailed,
}

impl Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::DigitalSelfTestFailed(n) => write!(f, "Digital startup self-test (index: {n}) failed"),
            Self::AnalogSelfTestFailed => write!(f, "Analog startup self-test failed"),
            Self::RepetitionCountTestFailed => write!(f, "Repetition count runtime self-test failed"),
            Self::AdaptiveProportionTestFailed => write!(f, "Adaptive proportion runtime self-test failed"),
        }
    }
}

impl core::error::Error for Error {}

/// True Random Number Generator (TRNG) Driver for MSPM0 series.
///
/// The driver provides blocking random numbers with [`TryRng`] methods and asynchronous counterparts in [`Trng::async_read_u32`], [`Trng::async_read_u64`], and [`Trng::async_read_bytes`].
///
/// The TRNG can be configured with different decimation rates. See [`DecimRate`], [`FastDecimRate`], and [`CryptoDecimRate`].
/// The TRNG can be instantiated with [`Trng::new`], [`Trng::new_fast`], or [`Trng::new_secure`].
///
/// Usage example, taking the peripheral from `embassy_mspm0::init`:
/// ```no_run
/// use embassy_mspm0::Peri;
/// use embassy_mspm0::peripherals::TRNG;
/// use embassy_mspm0::trng::Trng;
/// use rand_core::TryRng;
///
/// fn fill(peripheral: Peri<'static, TRNG>) {
///     let mut trng = Trng::new(peripheral).expect("Failed to initialize TRNG");
///     let mut randomness = [0u8; 16];
///
///     trng.try_fill_bytes(&mut randomness).unwrap();
///     assert_ne!(randomness, [0u8; 16]);
/// }
/// ```
pub struct Trng<'d, L: SecurityMarker> {
    inner: TrngInner<'d>,
    _security_level: PhantomData<L>,
}

impl<'d> Trng<'d, Crypto> {
    /// Setup a TRNG driver with the default safe decimation rate (Decim4).
    #[inline(always)]
    pub fn new(peripheral: Peri<'d, TRNG>) -> Result<Self, Error> {
        Self::new_secure(peripheral, CryptoDecimRate::Decim4)
    }
}

// Split off new methods to allow type inference when providing the decimation rate.
impl<'d> Trng<'d, Fast> {
    /// Setup a TRNG driver with a specific fast decimation rate.
    ///
    /// <div class="warning"> The created TRNG is not suitable for cryptography. Use <a href="struct.Trng.html#method.new_secure"><code>Trng::new_secure</code></a> instead. </div>
    #[inline(always)]
    pub fn new_fast(peripheral: Peri<'d, TRNG>, rate: FastDecimRate) -> Result<Self, Error> {
        Self::new_with_rate(peripheral, rate)
    }
}

impl<'d> Trng<'d, Crypto> {
    /// Setup a TRNG driver with a specific cryptographic decimation rate.
    #[inline(always)]
    pub fn new_secure(peripheral: Peri<'d, TRNG>, rate: CryptoDecimRate) -> Result<Self, Error> {
        Self::new_with_rate(peripheral, rate)
    }
}

impl<'d, D: SecurityMarker> Trng<'d, D> {
    #[inline(always)]
    fn new_with_rate(_peripheral: Peri<'d, TRNG>, rate: D::DecimRate) -> Result<Self, Error> {
        Ok(Self {
            inner: TrngInner::new(rate.into())?,
            _security_level: PhantomData,
        })
    }

    /// L-series TRM 13.2.5.2(10): procedure for recovering from a failed read.
    ///
    /// This method reinitializes the TRNG which may take some time to complete.
    #[inline(always)]
    pub fn fail_reset(&mut self) -> Result<(), Error> {
        self.inner.fail_reset()
    }

    /// Asynchronously read a 32-bit random value from the TRNG.
    ///
    /// The synchronous counterpart is given by [`TryRng::try_next_u32`].
    ///
    /// As with the [`synchronous`](TryRng) methods, an [`Err`] may be retried up to two times after calling [`Trng::fail_reset`].
    #[cfg(feature = "rt")]
    #[inline(always)]
    pub async fn async_read_u32(&mut self) -> Result<u32, Error> {
        self.inner.async_read_u32().await
    }

    /// Asynchronously read a 64-bit random value from the TRNG.
    ///
    /// The synchronous counterpart is given by [`TryRng::try_next_u64`].
    ///
    /// As with the [`synchronous`](TryRng) methods, an [`Err`] may be retried up to two times after calling [`Trng::fail_reset`].
    #[cfg(feature = "rt")]
    #[inline(always)]
    pub async fn async_read_u64(&mut self) -> Result<u64, Error> {
        self.inner.async_read_u64().await
    }

    /// Asynchronously fill `dest` with random bytes from the TRNG.
    ///
    /// The synchronous counterpart is given by [`TryRng::try_fill_bytes`].
    ///
    /// As with the [`synchronous`](TryRng) methods, an [`Err`] may be retried up to two times after calling [`Trng::fail_reset`].
    ///
    /// > **Note**
    /// When an error condition occurs, the buffer may be partially filled.
    #[cfg(feature = "rt")]
    #[inline(always)]
    pub async fn async_read_bytes(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        self.inner.async_read_bytes(dest).await
    }
}

/// Implements the fallible [`TryRng`].
///
/// If any of the methods give an [`Err`], the operation may be retried up to two times after calling [`Trng::fail_reset`].
impl<D: SecurityMarker> TryRng for Trng<'_, D> {
    type Error = Error;

    #[inline(always)]
    fn try_next_u32(&mut self) -> Result<u32, Error> {
        self.inner.try_next_u32()
    }

    #[inline(always)]
    fn try_next_u64(&mut self) -> Result<u64, Error> {
        self.inner.try_next_u64()
    }

    /// > **Note**
    /// When an error condition occurs, the buffer may be partially filled.
    #[inline(always)]
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        self.inner.try_fill_bytes(dest)
    }
}

impl TryCryptoRng for Trng<'_, Crypto> {}

// Inner TRNG driver implementation. Used to reduce monomorphization bloat.
struct TrngInner<'d> {
    decim_rate: vals::DecimRate,
    /// Held for as long as the driver exists; see
    /// [`SleepInfo::floor_to_keep_configured`](crate::sysctl::SleepInfo::floor_to_keep_configured).
    ///
    /// Deep sleep discards the TRNG's configuration entirely (L-series TRM 13.2.3). Dropping this in
    /// exchange for re-running `init()` on wake is the follow-up that would let a program with a TRNG
    /// still reach STANDBY.
    _retention_guard: Option<WakeGuard>,
    _phantom: PhantomData<&'d ()>,
}

impl TrngInner<'_> {
    fn new(decim_rate: vals::DecimRate) -> Result<Self, Error> {
        let mut trng = TrngInner {
            decim_rate: decim_rate,
            _retention_guard: <TRNG as LowPowerInstance>::SLEEP
                .floor_to_keep_configured()
                .map(WakeGuard::new),
            _phantom: PhantomData,
        };

        trng.reset();
        // L-series TRM 13.2.5.2
        trng.power_on(); // 1. Power on the TRNG peripheral.
        asm::delay(16); // Delay 16 cycles for power-on.
        trng.init()?; // 2-8. Initialize the TRNG.
        Ok(trng)
    }

    fn fail_reset(&mut self) -> Result<(), Error> {
        regs().iclr().write(|w| w.set_irq_health_fail(true));
        self.set_cmd(PwrOff);
        self.init()
    }

    fn init(&mut self) -> Result<(), Error> {
        // L-series TRM 13.2.5.2
        self.set_div(); // 2. Set the clock divider.
        regs().imask().write_value(Int::default()); // 3. Disable all interrupts.
        self.set_cmd(NormFunc); // 4. Set to normal function mode.
        self.dig_test()?; // 5. Perform digital block start-up self-tests.
        self.ana_test()?; // 6. Perform analog block start-up self-test.
        self.clr_rdy(); // 7.a Clear IRQ_CAPTURED_RDY_IRQ.
        self.set_decim_rate(); // 7.b Set decimation rate.
        self.set_cmd(NormFunc); // 7.b Set to normal function mode again after changing decimation rate.
        _ = self.read(); // 8. By 13.2.4.1, must discard first value.
        Ok(())
    }

    fn reset(&mut self) {
        regs().gprcm().rstctl().write(|w| {
            w.set_key(RstctlKey::Key);
            w.set_resetassert(true);
        });
    }

    fn power_on(&mut self) {
        regs().gprcm().pwren().write(|w| {
            w.set_key(PwrenKey::Key);
            w.set_enable(true);
        });
    }

    fn power_off(&mut self) {
        regs().gprcm().pwren().write(|w| w.set_key(PwrenKey::Key));
    }

    fn set_div(&mut self) {
        // MCLK now depends on the configured clock tree, so the band is picked at runtime. A tree
        // that leaves MCLK unusable here is a configuration error the user can only hit deliberately,
        // and the reset tree is checked at build time below.
        let mclk = crate::sysctl::clocks().mclk;
        let ratio = unwrap!(
            trng_ratio(mclk),
            "MCLK is outside the 9.5-20 MHz window the TRNG can be divided into"
        );

        regs().clkdiv().write(|w| w.set_ratio(ratio));
    }

    fn set_decim_rate(&mut self) {
        regs().ctl().modify(|w| w.set_decim_rate(self.decim_rate));
    }

    fn clr_rdy(&mut self) {
        regs().iclr().write(|w| w.set_irq_captured_rdy(true));
    }

    fn set_cmd(&mut self, cmd: vals::Cmd) {
        regs().iclr().write(|w| w.set_irq_cmd_done(true));
        regs().ctl().modify(|w| w.set_cmd(cmd));
        while !regs().ris().read().irq_cmd_done() {}
    }

    fn dig_test(&mut self) -> Result<(), Error> {
        self.set_cmd(PwrupDig);
        let results = regs().test_results().read();
        for n in 0..8u8 {
            // 13.2.4.1: Digital tests must pass.
            if !results.dig_test(n as usize) {
                return Err(Error::DigitalSelfTestFailed(n));
            }
        }
        Ok(())
    }

    fn ana_test(&mut self) -> Result<(), Error> {
        // 13.2.4.2: Analog tests have a small chance to fail, so try up to 3 times.
        for _ in 0..3 {
            self.set_cmd(PwrupAna);
            let results = regs().test_results().read();
            if !results.ana_test() {
                self.set_cmd(PwrOff);
            } else {
                return Ok(());
            }
        }
        Err(Error::AnalogSelfTestFailed)
    }

    fn poll(&mut self) -> Poll<Result<u32, Error>> {
        let ris = regs().ris().read();
        if ris.irq_health_fail() {
            // 10. The TRNG failed the health check.
            let stat = regs().stat().read();
            if stat.adap_fail() {
                Poll::Ready(Err(Error::AdaptiveProportionTestFailed))
            } else if stat.rep_fail() {
                Poll::Ready(Err(Error::RepetitionCountTestFailed))
            } else {
                unreachable!("Unexpected TRNG health failure type")
            }
        } else if ris.irq_captured_rdy() {
            // 9. When data is ready, read it.
            self.clr_rdy();
            let data = regs().data_capture().read();
            Poll::Ready(Ok(data))
        } else {
            Poll::Pending
        }
    }

    fn read(&mut self) -> Result<u32, Error> {
        loop {
            if let Poll::Ready(r) = self.poll() {
                return r;
            }
        }
    }

    #[cfg(feature = "rt")]
    async fn async_read_u32(&mut self) -> Result<u32, Error> {
        let _guard = <TRNG as LowPowerInstance>::SLEEP
            .power_domain
            .floor_to_keep_running(crate::sysctl::clocks().mclk)
            .map(WakeGuard::new);

        poll_fn(|cx| {
            WAKER.register(cx.waker());
            let result = self.poll();
            if result.is_pending() {
                regs().imask().write(|w| {
                    w.set_irq_captured_rdy(true);
                    w.set_irq_health_fail(true);
                });
            }
            result
        })
        .await
    }

    #[cfg(feature = "rt")]
    async fn async_read_u64(&mut self) -> Result<u64, Error> {
        let v1 = u64::from(self.async_read_u32().await?);
        let v2 = u64::from(self.async_read_u32().await?);
        Ok(v2 << 32 | v1)
    }

    #[cfg(feature = "rt")]
    async fn async_read_bytes(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        let mut left = dest;
        while left.len() >= 4 {
            let (l, r) = left.split_at_mut(4);
            left = r;
            let chunk = self.async_read_u32().await?.to_ne_bytes();
            l.copy_from_slice(&chunk);
        }
        if !left.is_empty() {
            let chunk = self.async_read_u32().await?.to_ne_bytes();
            left.copy_from_slice(&chunk[..left.len()]);
        }
        Ok(())
    }
}

impl Drop for TrngInner<'_> {
    fn drop(&mut self) {
        regs().imask().write_value(Int::default()); // Disable all interrupts.
        self.power_off();
    }
}

impl TryRng for TrngInner<'_> {
    type Error = Error;

    fn try_next_u32(&mut self) -> Result<u32, Error> {
        self.read()
    }

    fn try_next_u64(&mut self) -> Result<u64, Error> {
        let v1 = u64::from(self.try_next_u32()?);
        let v2 = u64::from(self.try_next_u32()?);
        Ok(v2 << 32 | v1)
    }

    /// > **Note**
    /// When an error condition occurs, the buffer may be partially filled.
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        let mut left = dest;
        while left.len() >= 4 {
            let (l, r) = left.split_at_mut(4);
            left = r;
            let chunk = self.try_next_u32()?.to_ne_bytes();
            l.copy_from_slice(&chunk);
        }
        if !left.is_empty() {
            let chunk = self.try_next_u32()?.to_ne_bytes();
            left.copy_from_slice(&chunk[..left.len()]);
        }
        Ok(())
    }
}

fn regs() -> crate::pac::trng::Trng {
    crate::pac::TRNG
}

/// `TRNGCLKF`, the range the clock reaching the module after `CLKDIV.RATIO` must stay within.
const CLK_MIN_HZ: u32 = crate::_generated::TRNG_CLK_MIN_HZ;
const CLK_MAX_HZ: u32 = crate::_generated::TRNG_CLK_MAX_HZ;

/// Divider that brings `mclk_hz` into the TRNG's input window, or `None` if none does.
///
/// The TRNG is derived from MCLK (SLAU847 13.2.2). The window is the datasheet's, which is wider than
/// the 9.5-20 MHz that TRM quotes.
const fn trng_ratio(mclk_hz: u32) -> Option<Ratio> {
    // Largest divider first, so each band is the smallest division that lands inside the window.
    if mclk_hz >= 8 * CLK_MIN_HZ {
        Some(Ratio::DivBy8)
    } else if mclk_hz >= 6 * CLK_MIN_HZ {
        Some(Ratio::DivBy6)
    } else if mclk_hz >= 4 * CLK_MIN_HZ {
        Some(Ratio::DivBy4)
    } else if mclk_hz >= 2 * CLK_MIN_HZ {
        Some(Ratio::DivBy2)
    } else if mclk_hz >= CLK_MIN_HZ {
        Some(Ratio::DivBy1)
    } else {
        None
    }
}

/// Whether dividing `mclk_hz` by `divisor` lands inside the TRNG's input window.
const fn divides_into_window(mclk_hz: u32, divisor: u32) -> bool {
    let trngclk = mclk_hz / divisor;

    trngclk >= CLK_MIN_HZ && trngclk <= CLK_MAX_HZ
}

const _: () = {
    core::assert!(
        crate::sysctl::MAX_MCLK_HZ <= 8 * CLK_MAX_HZ,
        "MCLK can exceed what the largest TRNG divider brings into range"
    );

    // The tree the device boots with must always be usable, so only a deliberately reconfigured
    // clock tree can reach the runtime failure in `set_div`.
    core::assert!(
        trng_ratio(crate::sysctl::Clocks::RESET.mclk).is_some(),
        "the reset MCLK is outside the TRNG's input range"
    );

    // Below the window nothing helps, since dividing can only go lower.
    core::assert!(matches!(trng_ratio(CLK_MIN_HZ - 1), None));

    // Each band picks the divider it should, and that divider lands inside the window. Checked at
    // the band's floor, which is where its divided rate is lowest — the thresholds themselves are
    // per device now, so asserting them would just restate `trng_ratio`.
    core::assert!(matches!(trng_ratio(CLK_MIN_HZ), Some(Ratio::DivBy1)) && divides_into_window(CLK_MIN_HZ, 1));
    core::assert!(matches!(trng_ratio(2 * CLK_MIN_HZ), Some(Ratio::DivBy2)) && divides_into_window(2 * CLK_MIN_HZ, 2));
    core::assert!(matches!(trng_ratio(4 * CLK_MIN_HZ), Some(Ratio::DivBy4)) && divides_into_window(4 * CLK_MIN_HZ, 4));
    core::assert!(matches!(trng_ratio(6 * CLK_MIN_HZ), Some(Ratio::DivBy6)) && divides_into_window(6 * CLK_MIN_HZ, 6));
    core::assert!(matches!(trng_ratio(8 * CLK_MIN_HZ), Some(Ratio::DivBy8)) && divides_into_window(8 * CLK_MIN_HZ, 8));
};

// This symbol is weakly defined as DefaultHandler and is called by the interrupt group implementation.
// Defining this as no_mangle is required so that the linker will pick this up.
#[cfg(feature = "rt")]
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
fn TRNG() {
    regs().imask().write_value(Int::default()); // Disable all interrupts.
    WAKER.wake();
}
