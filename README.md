# OSPI slave with AXI4 master backend

This repository implements the custom 8-bit SDR protocol defined by
`doc/OSPI_Specification.pdf`. The slave captures OSPI requests and transfers
ICCM/DCCM data through an AXI4 master interface in the `clk` domain.

## RTL

- `rtl/ospi_slave_top.v`: top-level wrapper, module `ospi_slave_top`.
- `rtl/io_top.v`: synthesizable RTL pad model for input, bidirectional,
  input-enable, output-enable and High-Z behavior.
- `rtl/ospi_slave.v`: pure-digital module `ospi_slave`, containing the OSPI
  protocol engine, CDC FIFOs and AXI4 master; it has no `inout` ports or
  High-Z assignments.
- `rtl/async_fifo.v`: function-free Gray-pointer asynchronous FIFO.

`ospi_slave_top` instantiates `io_top` and the pure-digital `ospi_slave` core.
During ASIC integration, `io_top` can be replaced with technology-specific IO
cells without modifying the core.

The `D[7:0]` input buffer is enabled only during Command, Address and Write
phases. When disabled, the core sees `8'h00`; the physical pad still exposes
electrical contention as `X` in RTL simulation. `D[7:0]` is driven only during
Read data and is High-Z while unselected. Dedicated `clk`, `rst_n`, `SCLK` and
`CSN` input buffers are permanently enabled.

`SRDY` uses a bidirectional pad model with its input buffer disabled. In
push-pull mode it always drives the required logic level. In open-drain mode it
only drives low while selected and busy, and otherwise releases High-Z.

The Request FIFO is internal and two entries deep. Its 40-bit word is formed
inside `ospi_slave` directly from the captured command and address signals:

```text
{write, length_code, 4'b0, aligned_address}
```

The top level no longer exposes Request, Write-data or Read-data FIFO pins.

## AXI4 interface

- Clock: `clk`
- Address width: 32 bits
- Data width: parameter `AXI_DATA_WIDTH`, default 32 bits
- ID width: parameter `AXI_ID_WIDTH`, default 6 bits
- ID value: parameter `AXI_ID`, default zero
- Protection: parameter `AXI_PROT`, default `3'b001`
- Burst type: INCR
- Transfer size: 32-bit narrow AXI beats, matching the OSPI protocol word

`AXI_DATA_WIDTH` must be a power-of-two multiple of 32. On an AXI bus wider
than 32 bits, address bits select the proper `WDATA`, `WSTRB` and `RDATA` byte
lanes.

`SINGLE_TRAN` defaults to 1, so each OSPI 32-bit word is issued as a separate
AXI transaction with `AWLEN/ARLEN=0`. Define `SINGLE_TRAN=0` to issue all words
in an OSPI request as one AXI INCR burst. OSPI lengths from 4 through 512 bytes
map to 1 through 128 AXI beats.

AXI read data is buffered in the internal Read-data FIFO. If AXI data is not
available, the OSPI slave holds `SRDY` low. OSPI write data is buffered in the
internal Write-data FIFO and AXI backpressure propagates to `SRDY` if it fills.

## Verification

```sh
cd sim
make clean all
```

The test suite covers the RTL pad model and IE/OE/High-Z behavior, 32-entry and
two-entry FIFO configurations, AXI single transactions, AXI bursts, OSPI
Write-to-AXI memory, AXI-memory-to-OSPI Read, byte ordering, address use and
AXI attributes.

All synthesizable RTL is Verilog-2005 and contains no `function` or
`always_ff` constructs.
