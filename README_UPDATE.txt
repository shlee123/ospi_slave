OSPI master register-map update package
=======================================

Source basis:
- Current merged OSPI master/testbench behavior from shlee123/ospi_slave main
  (same functional model used for the previously green PR regression).
- Register specification: ospi_master_register(2).xlsx

Functional changes in this package:
1. Program Mode base address changed:
     OLD: 0xB000_B000
     NEW: 0x0FFF_0000

2. Program Mode absolute register addresses are now:
     Cmd   : 0x0FFF_0000
     Addr  : 0x0FFF_0004
     Wdata : 0x0FFF_0008
     Rdata : 0x0FFF_000C
     Ctrl  : 0x0FFF_0010

3. Direct Mode mapping is unchanged:
     0x1000_0000..0x2FFF_FFFF -> CSN0
     0x3000_0000..0x4FFF_FFFF -> CSN1
     0x5000_0000..0x6FFF_FFFF -> CSN2
     0x7000_0000..0x8FFF_FFFF -> CSN3

4. Existing functional behavior is preserved:
   - Direct Write requires PSTRB=4'b1111.
   - Unaligned Direct access returns PSLVERR.
   - Direct Read returns data on PRDATA.
   - Direct Write uses PWDATA directly.
   - OSPI byte order is MSB first.
   - SRDY=0 holds the current byte and retries on following SCLK rising edges.
   - Default APB wait timeout is 1024 PCLK cycles.
   - Program Mode TX/RX FIFO depth remains 8 x 32-bit.
   - Writing Cmd starts a Program Mode transaction.

Files changed:
- model/ospi_master.v
- sim/tb_apb_ospi.v

Reference included:
- reference/ospi_master_register(2).xlsx

Notes:
- The spreadsheet's pin-description rows contain names/directions such as
  PSTRB/PROT/PSELEVRR that do not match the existing APB slave-side model
  interface. Because this request was specifically a register-map adjustment,
  the APB pin interface was intentionally not changed.
- No local Icarus/Verilator executable is available in this sandbox, so the
  package was structurally checked but not locally simulated. Run the existing
  GitHub regression after applying these two files.
