//! Inter-Integrated-Circuit (I2C) Target
// The following code is modified from embassy-stm32 and embassy-rp
// https://github.com/embassy-rs/embassy/tree/main/embassy-stm32
// https://github.com/embassy-rs/embassy/tree/main/embassy-rp

use core::future::poll_fn;
use core::task::Poll;

use embassy_embedded_hal::SetConfig;
use mspm0_metapac::i2c::vals::CpuIntIidxStat;

use crate::gpio::{AnyPin, SealedPin};
use crate::i2c::{Address, ClockSel, ConfigError, Info, Instance, InterruptHandler, SclPin, SdaPin, State};
use crate::interrupt::InterruptExt;
use crate::pac::i2c::vals;
use crate::pac::{self};
use crate::sysctl::MaybeWakeGuard;
use crate::{Peri, i2c, i2c_target, interrupt};

/// A second address for the target to answer on, with the bits of it to ignore.
///
/// 7-bit only, and only alongside a 7-bit [`Config::target_addr`]: `OAR2` is compared just while the
/// target is in 7-bit mode, so pairing it with a 10-bit primary address is rejected with
/// [`ConfigError::SecondAddressWith10Bit`] rather than accepted and never matched.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SecondAddress {
    /// The address to answer on, 7-bit.
    pub addr: u8,

    /// Bits set here are not compared, so the target answers a range of addresses rather than one.
    ///
    /// `0` matches [`Self::addr`] alone. Which address in the range a command arrived on is
    /// [`I2cTarget::matched_address`].
    ///
    /// A range covering `0x00` reports its commands as [`Command::GeneralCall`], that address meaning
    /// exactly that on the wire.
    pub mask: u8,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// How an [`I2cTarget`] answers the bus.
pub struct Config {
    /// Target address to answer on.
    pub target_addr: Address,

    /// A second address to answer on, alongside [`Self::target_addr`].
    pub second_addr: Option<SecondAddress>,

    /// Control if the target should ack to and report general calls.
    pub general_call: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            target_addr: Address::SevenBit(0x48),
            second_addr: None,
            general_call: false,
        }
    }
}

/// I2C error
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// User passed in a response buffer that was 0 length
    InvalidResponseBufferLength,
    /// The response buffer length was too short to contain the message
    ///
    /// The length parameter will always be the length of the buffer, and is
    /// provided as a convenience for matching alongside `Command::Write`.
    PartialWrite(usize),
    /// The response buffer length was too short to contain the message
    ///
    /// The length parameter will always be the length of the buffer, and is
    /// provided as a convenience for matching alongside `Command::GeneralCall`.
    PartialGeneralCall(usize),
}

/// Received command from the controller.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// General Call Write: Controller sent the General Call address (0x00) followed by data.
    /// Contains the number of bytes written by the controller.
    GeneralCall(usize),
    /// Read: Controller wants to read data from the target.
    Read,
    /// Write: Controller sent the target's address followed by data.
    /// Contains the number of bytes written by the controller.
    Write(usize),
    /// Write followed by Read (Repeated Start): Controller wrote data, then issued a repeated
    /// start and wants to read data. Contains the number of bytes written before the read.
    ///
    /// **A 10-bit controller re-sends the whole address between the halves**, so the frame before the
    /// read carries no data and looks like the one a plain 10-bit read opens with. Whether such a
    /// transaction arrives here or as a [`Command::Read`] has not been measured; do not rely on
    /// either at 10-bit.
    WriteRead(usize),
}

/// Status after responding to a controller read request.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadStatus {
    /// Transaction completed successfully. The controller either NACKed the last byte
    /// or sent a STOP condition.
    Done,
    /// Transaction incomplete, controller trying to read more bytes than were provided
    NeedMoreBytes,
    /// Transaction complete, but controller stopped reading bytes before we ran out
    LeftoverBytes(u16),
}

/// I2C Target driver.
// Use the same Instance, SclPin, SdaPin traits as the controller
pub struct I2cTarget<'d> {
    info: &'static Info,
    state: &'static State,
    scl: Option<Peri<'d, AnyPin>>,
    sda: Option<Peri<'d, AnyPin>>,

    /// The clock source, divider and rate this instance was configured for.
    ///
    /// Derived once, when the configuration arrives, so `init` programs the registers from one
    /// answer rather than re-deriving its own.
    resolved: i2c::Resolved,

    target_config: i2c_target::Config,
    wake_guard: MaybeWakeGuard,
}

impl<'d> SetConfig for I2cTarget<'d> {
    type Config = (i2c::Config, i2c_target::Config);
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.info.interrupt.disable();

        if let Some(ref sda) = self.sda {
            sda.update_pf(config.0.sda_pf());
        }

        if let Some(ref scl) = self.scl {
            scl.update_pf(config.0.scl_pf());
        }

        self.resolved = config.0.resolve()?;
        self.target_config = config.1;

        self.reset()
    }
}

impl<'d> I2cTarget<'d> {
    /// Create a new asynchronous I2C target driver using interrupts
    /// The `config` reuses the i2c controller config to setup the clock while `target_config`
    /// configures i2c target specific parameters.
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: i2c::Config,
        target_config: i2c_target::Config,
    ) -> Result<Self, ConfigError> {
        let mut this = Self::new_inner(
            peri,
            new_pin!(scl, config.scl_pf()),
            new_pin!(sda, config.sda_pf()),
            config,
            target_config,
        )?;
        this.reset()?;
        Ok(this)
    }

    /// Reset the i2c peripheral. If you cancel a respond_to_read, you may stall the bus.
    /// You can recover the bus by calling this function, but doing so will almost certainly cause
    /// an i/o error in the controller.
    pub fn reset(&mut self) -> Result<(), ConfigError> {
        self.init()?;
        unsafe { self.info.interrupt.enable() };

        self.wake_guard = MaybeWakeGuard::new(self.resolved.wake_floor(&self.info.sleep));
        Ok(())
    }
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Option<Peri<'d, AnyPin>>,
        sda: Option<Peri<'d, AnyPin>>,
        config: i2c::Config,
        target_config: i2c_target::Config,
    ) -> Result<Self, ConfigError> {
        let resolved = config.resolve()?;

        if let Some(ref scl) = scl {
            let pincm = pac::IOMUX.pincm(scl._pin_cm() as usize);
            pincm.modify(|w| {
                w.set_hiz1(true);
            });
        }
        if let Some(ref sda) = sda {
            let pincm = pac::IOMUX.pincm(sda._pin_cm() as usize);
            pincm.modify(|w| {
                w.set_hiz1(true);
            });
        }

        Ok(Self {
            info: T::info(),
            state: T::state(),
            scl,
            sda,
            resolved,
            target_config,
            wake_guard: MaybeWakeGuard::none(),
        })
    }

    fn init(&mut self) -> Result<(), ConfigError> {
        let resolved = self.resolved;
        let target_config = self.target_config;
        let regs = self.info.regs;

        if !target_config.target_addr.fits() {
            return Err(ConfigError::InvalidTargetAddress);
        }

        if let Some(second) = target_config.second_addr {
            // Both `TOAR2` fields are seven bits wide, so anything larger would be answered on a
            // truncated address rather than rejected.
            if second.addr >= 0x80 || second.mask >= 0x80 {
                return Err(ConfigError::InvalidTargetAddress);
            }

            if matches!(target_config.target_addr, Address::TenBit(_)) {
                return Err(ConfigError::SecondAddressWith10Bit);
            }
        }

        regs.gprcm(0).rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(vals::ResetKey::Key);
        });

        regs.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });

        self.info.interrupt.disable();

        // Init delay from the M0 examples by TI in CCStudio (16 cycles)
        cortex_m::asm::delay(16);

        regs.clksel().write(|w| match resolved.clock_source {
            ClockSel::BusClk => {
                w.set_mfclk_sel(false);
                w.set_busclk_sel(true);
            }
            ClockSel::MfClk => {
                w.set_mfclk_sel(true);
                w.set_busclk_sel(false);
            }
        });
        regs.clkdiv().write(|w| w.set_ratio(resolved.clock_div.into()));

        regs.target(0).toar().modify(|w| {
            w.set_oaren(true);
            w.set_oar(target_config.target_addr.addr());
            w.set_tmode(target_config.target_addr.mode());
        });

        // Written whether or not a second address was asked for, so that reconfiguring without one
        // disables the address the previous configuration answered on.
        regs.target(0).toar2().write(|w| {
            if let Some(second) = target_config.second_addr {
                w.set_oar2en(true);
                w.set_oar2(second.addr);
                w.set_oar2_mask(second.mask);
            }
        });

        regs.target(0).tctr().modify(|w| {
            w.set_gencall(target_config.general_call);
            w.set_tclkstretch(true);
            // Disable target wakeup, follow TI example. (TI note: Workaround for errata I2C_ERR_04.)
            w.set_twuen(false);
            w.set_txempty_on_treq(true);
            // What a read left unsent must not answer the next one. Set, the transmit state machine
            // is told the FIFO is empty at every STOP, restart and timeout whether or not bytes are
            // still in it, so the surplus is never shifted out; `flush_stale_tx_fifo` then clears it
            // once the next read stretches the clock. SLAU846 §25.2.3.13.1.
            //
            // Software alone cannot do this. A flush between two commands races a controller that
            // starts reading immediately after the STOP, and loses: only the state machine can
            // refuse to transmit. It needs `TXEMPTY_ON_TREQ` above, or the stretch raises nothing.
            w.set_txwait_stale_txfifo(true);
        });

        regs.target(0).tctr().modify(|w| {
            w.set_active(true);
        });

        Ok(())
    }

    #[inline(always)]
    fn drain_fifo(&mut self, buffer: &mut [u8], offset: &mut usize) {
        let regs = self.info.regs;

        for b in &mut buffer[*offset..] {
            if regs.target(0).tfifosr().read().rxfifocnt() == 0 {
                break;
            }

            *b = regs.target(0).trxdata().read().value();
            *offset += 1;
        }
    }

    /// Discard whatever a finished command left in a FIFO.
    ///
    /// SLAU846 §25.2.3.13 asks for the FIFO interrupts to be masked before a flush and their flags to
    /// be dealt with after, and both matter here rather than being ceremony: emptying a FIFO raises
    /// the same events a finished command does, and left latched they are answered by the next
    /// [`I2cTarget::listen`] — which would read a flushed transmit FIFO as a fresh `Command::Read`.
    ///
    /// The controller's `I2c::flush_fifos` is the same routine against the other half of the
    /// peripheral; fix one and look at the other.
    fn flush_fifos(&mut self, tx: bool, rx: bool) {
        let target = self.info.regs.target(0);
        let int = self.info.regs.cpu_int(0);

        // Read back and restored one field at a time rather than saved and rewritten whole, so a
        // change to any other bit between here and the end of the flush survives it.
        let armed = int.imask().read();
        int.imask().modify(|w| {
            w.set_ttxfifotrg(false);
            w.set_trxfifotrg(false);
            w.set_ttxempty(false);
            w.set_trxfifofull(false);
        });

        target.tfifoctl().modify(|w| {
            w.set_txflush(tx);
            w.set_rxflush(rx);
        });
        // Unbounded, and deliberately so: this waits on the FIFO emptying itself with the flush bits
        // held, which is the peripheral's own doing and does not depend on the bus.
        while (tx && target.tfifosr().read().txfifocnt() as usize != self.info.fifo_size)
            || (rx && target.tfifosr().read().rxfifocnt() != 0)
        {}
        target.tfifoctl().modify(|w| {
            w.set_txflush(false);
            w.set_rxflush(false);
        });

        int.iclr().write(|w| {
            w.set_ttxfifotrg(true);
            w.set_trxfifotrg(true);
            w.set_ttxempty(true);
            w.set_trxfifofull(true);
        });
        int.imask().modify(|w| {
            w.set_ttxfifotrg(armed.ttxfifotrg());
            w.set_trxfifotrg(armed.trxfifotrg());
            w.set_ttxempty(armed.ttxempty());
            w.set_trxfifofull(armed.trxfifofull());
        });
    }

    /// Discard whatever is left in the receive FIFO.
    ///
    /// Used where a command ends with the controller still sending: the bytes that did not fit stay
    /// queued otherwise, and the next [`I2cTarget::listen`] hands them over as the start of the
    /// following write.
    #[inline]
    fn flush_rx_fifo(&mut self) {
        self.flush_fifos(false, true);
    }

    /// Discard a previous read's unsent bytes, if the peripheral says any are left.
    ///
    /// SLAU846 §25.2.3.13.1's step 4. `TXWAIT_STALE_TXFIFO` keeps the surplus off the wire but does
    /// not remove it, so it stays in the way until something empties it; this is what does.
    fn flush_stale_tx_fifo(&mut self) {
        if self.info.regs.target(0).tsr().read().stale_txfifo() {
            self.flush_fifos(true, false);
        }
    }

    /// The address the last command was addressed to.
    ///
    /// Worth asking only with a masked [`SecondAddress`], where the controller's address is one of a
    /// range rather than the one that was configured. The peripheral re-evaluates this on every address
    /// comparison, so it answers for the last command and not for any earlier one.
    pub fn matched_address(&self) -> Address {
        let tsr = self.info.regs.target(0).tsr().read();

        // `TOAR2` has no mode bit, so a match against the second address is 7-bit even on a target whose
        // primary address is not.
        if tsr.oar2sel() || matches!(self.target_config.target_addr, Address::SevenBit(_)) {
            Address::SevenBit(tsr.addrmatch() as u8)
        } else {
            Address::TenBit(tsr.addrmatch())
        }
    }

    /// Whether the last command was addressed to [`Config::second_addr`] rather than the primary address.
    pub fn matched_second_address(&self) -> bool {
        self.info.regs.target(0).tsr().read().oar2sel()
    }

    /// Discard whatever is queued to transmit, blocking until it is gone.
    ///
    /// Calling this after a [`ReadStatus::LeftoverBytes`] is no longer needed: the surplus cannot
    /// reach the bus, and the next read clears it. It remains for a caller who wants the queue empty
    /// at a moment of their own choosing — after preparing a response the controller never came back
    /// for, say.
    pub fn flush_tx_fifo(&mut self) {
        self.flush_fifos(true, false);
    }
    /// Wait asynchronously for commands from an I2C controller.
    /// `buffer` is provided in case controller does a 'write', 'write read', or 'general call' and is unused for 'read'.
    pub async fn listen(&mut self, buffer: &mut [u8]) -> Result<Command, Error> {
        let regs = self.info.regs;

        let mut len = 0;

        // Set the rx fifo interrupt to avoid a fifo overflow
        regs.target(0).tfifoctl().modify(|r| {
            r.set_rxtrig(vals::TfifoctlRxtrig::Level6);
        });

        self.wait_on(
            |me| {
                // Check if address matches the General Call address (0x00)
                let is_gencall = regs.target(0).tsr().read().addrmatch() == 0;

                if regs.target(0).tfifosr().read().rxfifocnt() > 0 {
                    me.drain_fifo(buffer, &mut len);
                }

                if buffer.len() == len && regs.target(0).tfifosr().read().rxfifocnt() > 0 {
                    // Ending here still ends the command, so it owes the same cleanup as every other
                    // terminating arm below: disarm, and drop what did not fit. Returning without either
                    // leaves the surplus queued, and the next `listen` delivers it as the head of the
                    // following write — a corruption that surfaces one transaction later.
                    me.flush_rx_fifo();
                    regs.cpu_int(0).imask().write(|_| {});

                    if is_gencall {
                        return Poll::Ready(Err(Error::PartialGeneralCall(buffer.len())));
                    } else {
                        return Poll::Ready(Err(Error::PartialWrite(buffer.len())));
                    }
                }

                let iidx = regs.cpu_int(0).iidx().read().stat();
                trace!("ls:{} len:{}", iidx.to_bits(), len);
                let result = match iidx {
                    CpuIntIidxStat::Ttxempty => match len {
                        0 => Poll::Ready(Ok(Command::Read)),
                        w => Poll::Ready(Ok(Command::WriteRead(w))),
                    },
                    CpuIntIidxStat::Tstopfg => match (is_gencall, len) {
                        (_, 0) => Poll::Pending,
                        (true, w) => Poll::Ready(Ok(Command::GeneralCall(w))),
                        (false, w) => Poll::Ready(Ok(Command::Write(w))),
                    },
                    _ => Poll::Pending,
                };
                if !result.is_pending() {
                    regs.cpu_int(0).imask().write(|_| {});
                }
                result
            },
            |_me| {
                regs.cpu_int(0).imask().write(|_| {});
                regs.cpu_int(0).imask().modify(|w| {
                    w.set_tgencall(true);
                    w.set_trxfifotrg(true);
                    w.set_tstop(true);
                    w.set_ttxempty(true);
                });
            },
        )
        .await
    }

    /// Respond to an I2C controller 'read' command, asynchronously.
    pub async fn respond_to_read(&mut self, buffer: &[u8]) -> Result<ReadStatus, Error> {
        if buffer.is_empty() {
            return Err(Error::InvalidResponseBufferLength);
        }

        // Before a byte of this response is queued, not after: whatever the last read did not send is
        // still sitting in front of it, and appending to it would put this response behind the
        // previous one's tail.
        self.flush_stale_tx_fifo();

        let regs = self.info.regs;
        let fifo_size = self.info.fifo_size;
        let mut chunks = buffer.chunks(self.info.fifo_size);

        self.wait_on(
            |_me| {
                if let Some(chunk) = chunks.next() {
                    for byte in chunk {
                        regs.target(0).ttxdata().write(|w| w.set_value(*byte));
                    }

                    return Poll::Pending;
                }

                let iidx = regs.cpu_int(0).iidx().read().stat();
                let fifo_bytes = fifo_size - regs.target(0).tfifosr().read().txfifocnt() as usize;
                trace!("rs:{}, fifo:{}", iidx.to_bits(), fifo_bytes);

                let result = match iidx {
                    CpuIntIidxStat::Ttxempty => Poll::Ready(Ok(ReadStatus::NeedMoreBytes)),
                    CpuIntIidxStat::Tstopfg => match fifo_bytes {
                        0 => Poll::Ready(Ok(ReadStatus::Done)),
                        w => Poll::Ready(Ok(ReadStatus::LeftoverBytes(w as u16))),
                    },
                    _ => Poll::Pending,
                };
                if !result.is_pending() {
                    regs.cpu_int(0).imask().write(|_| {});
                }
                result
            },
            |_me| {
                regs.cpu_int(0).imask().write(|_| {});
                regs.cpu_int(0).imask().modify(|w| {
                    w.set_ttxempty(true);
                    w.set_tstop(true);
                });
            },
        )
        .await
    }

    /// Respond to reads with the fill byte until the controller stops asking
    pub async fn respond_till_stop(&mut self, fill: u8) -> Result<(), Error> {
        // The buffer size could be increased to reduce interrupt noise but has higher probability
        // of LeftoverBytes
        let buff = [fill];
        loop {
            match self.respond_to_read(&buff).await {
                Ok(ReadStatus::NeedMoreBytes) => (),
                Ok(_) => break Ok(()),
                Err(e) => break Err(e),
            }
        }
    }

    /// Respond to a controller read, then fill any remaining read bytes with `fill`
    pub async fn respond_and_fill(&mut self, buffer: &[u8], fill: u8) -> Result<ReadStatus, Error> {
        let resp_stat = self.respond_to_read(buffer).await?;

        if resp_stat == ReadStatus::NeedMoreBytes {
            self.respond_till_stop(fill).await?;
            Ok(ReadStatus::Done)
        } else {
            Ok(resp_stat)
        }
    }

    /// Calls `f` to check if we are ready or not.
    /// If not, `g` is called once(to eg enable the required interrupts).
    /// The waker will always be registered prior to calling `f`.
    #[inline(always)]
    async fn wait_on<F, U, G>(&mut self, mut f: F, mut g: G) -> U
    where
        F: FnMut(&mut Self) -> Poll<U>,
        G: FnMut(&mut Self),
    {
        poll_fn(|cx| {
            // Register prior to checking the condition
            self.state.waker.register(cx.waker());
            let r = f(self);

            if r.is_pending() {
                g(self);
            }

            r
        })
        .await
    }
}

impl<'d> Drop for I2cTarget<'d> {
    fn drop(&mut self) {
        // Ensure peripheral is disabled and pins are reset
        self.info.regs.target(0).tctr().modify(|w| w.set_active(false));

        self.scl.as_ref().map(|x| x.set_as_disconnected());
        self.sda.as_ref().map(|x| x.set_as_disconnected());
    }
}
