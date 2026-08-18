OSPI master simulation update package

Replace / add:
  model/ospi_master.v
  sim/tb_apb_ospi.v
  sim/Makefile

No functional change required:
  sim/filelist.f (included for convenience)

Existing testbenches tb_async_fifo.v, tb_io_top.v and tb_ospi_slave.v do not
instantiate ospi_master and therefore do not require changes for the new
4-bit CSN / Program Mode base / Direct Mode mapping.

Suggested regression:
  cd sim
  make run-apb-ospi SIMULATOR=iverilog
or
  make run-apb-ospi SIMULATOR=vcs

Debug behavior:
  STOP_ON_ERROR=1  (default): stop at first error
  STOP_ON_ERROR=0: accumulate errors until regression completion
