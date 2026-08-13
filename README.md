# OSPI slave RTL

This deliverable implements the custom 8-bit SDR OSPI slave defined by
`OSPI_Specification(10).pdf`.

## Files

- `rtl/ospi_slave.v`: OSPI protocol engine and three clock-domain crossing
  FIFOs.
- `rtl/async_fifo.v`: function-free dual-clock FIFO, default 32 entries.
- `sim/tb_ospi_slave.v`: Write, Read, address-alignment, byte-order and
  unsupported-command tests.
- `sim/Makefile`: Verilog-2005 syntax and simulation targets.

## Backend interface

All backend ports use `clk`. Each FIFO read output is registered and changes
on the `clk` rising edge accepting its read enable.

`req_data[39:0]` is `{write, length_code, 4'b0, effective_address}`. The system
reads requests using `req_rd_en`. Write words are read using `wr_rd_en`. For a
Read request, system logic writes the requested words through
`rd_wr_en/rd_wr_data`, observing `rd_full`.

The default FIFO depth is 32 words. The 512-byte protocol transfer is streamed
through the FIFO and does not require all 128 words to be resident at once.

## Simulation

```sh
cd sim
make clean all
```

The RTL is Verilog-2005 and contains no `function` or `always_ff` constructs.
