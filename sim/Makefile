SIM ?= iverilog
RTL := ../rtl/async_fifo.v ../rtl/ospi_slave.v

.PHONY: all run-ospi run-fifo syntax clean

all: run-fifo run-ospi

syntax:
	$(SIM) -g2005 -s ospi_slave -o ospi_syntax $(RTL)

run-fifo:
	$(SIM) -g2005 -s tb_async_fifo -o simv_fifo \
		../rtl/async_fifo.v tb_async_fifo.v
	vvp simv_fifo

run-ospi:
	$(SIM) -g2005 -s tb_ospi_slave -o simv_ospi $(RTL) tb_ospi_slave.v
	vvp simv_ospi

clean:
	rm -f simv simv_fifo simv_ospi ospi_syntax *.vcd *.fst *.fsdb *.log
