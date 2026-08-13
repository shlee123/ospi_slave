SIM ?= iverilog
RTL := ../rtl/async_fifo.v ../rtl/ospi_slave.v

.PHONY: all run-ospi-single run-ospi-burst run-fifo syntax clean

all: run-fifo run-ospi-single run-ospi-burst

syntax:
	$(SIM) -g2005 -s ospi_slave -o ospi_syntax $(RTL)

run-fifo:
	$(SIM) -g2005 -s tb_async_fifo -o simv_fifo \
		../rtl/async_fifo.v tb_async_fifo.v
	vvp simv_fifo

run-ospi-single:
	$(SIM) -g2005 -s tb_ospi_slave -o simv_ospi_single \
		$(RTL) tb_ospi_slave.v
	vvp simv_ospi_single

run-ospi-burst:
	$(SIM) -g2005 -DSINGLE_TRAN=0 -s tb_ospi_slave -o simv_ospi_burst \
		$(RTL) tb_ospi_slave.v
	vvp simv_ospi_burst

clean:
	rm -f simv simv_fifo simv_ospi simv_ospi_single simv_ospi_burst \
		ospi_syntax *.vcd *.fst *.fsdb *.log
