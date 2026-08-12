# ospi_slave

Custom OSPI protocol slave controller implemented in Verilog.

## Project goal

This repository develops a reusable OSPI slave controller, including RTL,
simulation testbenches, protocol verification, and implementation support.

## Directory structure

- `rtl/`: synthesizable design code
- `sim/`: simulation testbenches and run environment
- `model/`: reference and behavioral models

## Current RTL

`rtl/async_fifo.v` provides a dual-clock asynchronous FIFO with parameterized
data width and depth. Defaults are 8-bit data and 32 entries. `FIFO_DEPTH`
must be a power of two.

Run the smoke test with Icarus Verilog:

```sh
cd sim
make
```

## Status

Asynchronous FIFO added as the first reusable OSPI controller component.
