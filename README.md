# OSPI slave with AXI4 master backend

This repository implements the custom 8-bit SDR protocol defined by
`doc/OSPI_Specification.pdf`. The slave captures OSPI requests and transfers
ICCM/DCCM data through an AXI4 master interface in the `clk` domain.

## RTL

- `rtl/ospi_slave.v`: OSPI protocol engine, CDC FIFOs and AXI4 master.
- `rtl/async_fifo.v`: function-free Gray-pointer asynchronous FIFO.

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

The test suite covers the 32-entry and two-entry FIFO configurations, AXI
single transactions, AXI bursts, OSPI Write-to-AXI memory, AXI-memory-to-OSPI
Read, byte ordering, address use and AXI attributes.

All synthesizable RTL is Verilog-2005 and contains no `function` or
`always_ff` constructs.
