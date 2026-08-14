# Model

Reference and behavioral models for the OSPI slave controller belong here.

## `ospi_master.v`

`ospi_master` is a simulation-only APB4-to-OSPI behavioral model. Its APB
address and data widths are fixed at 32 bits. Every aligned full-word APB Read
or Write is translated into one 4-byte OSPI request using the existing
MSB-first command, address and data ordering.

The APB interface includes `PSTRB`, `PPROT` and `PSLVERR`. Partial writes and
unaligned accesses are rejected with `PSLVERR`. `PPROT` is accepted and
recorded by the model, but is not sent because the current OSPI packet has no
protection field. A request that does not finish within `TIMEOUT_CYCLES` PCLK
cycles is aborted and also completes with `PSLVERR`.

The model generates OSPI timing with delay controls. `SCLK_HALF_PERIOD`
specifies the half-period in the active simulation timescale; consequently the
model is not synthesizable.
