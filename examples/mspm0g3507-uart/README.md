# MSPM0G3507 UART instrumentation

Measurements of the buffered UART driver's interrupt path, kept apart from `examples/mspm0g3507` because
they need `embassy-mspm0`'s `_probe` feature and a cargo feature is crate-wide. An unarmed marker is only
a load and a branch, but it is a load and a branch in every interrupt handler in the crate, and the other
examples are where the timing results come from.

Each binary carries its own analyser channel map in its module doc.
