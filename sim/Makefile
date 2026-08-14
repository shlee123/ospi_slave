SIM ?= iverilog
VVP ?= vvp
RTL := ../rtl/async_fifo.v ../rtl/io_top.v ../rtl/ospi_slave.v \
	../rtl/ospi_slave_top.v

.PHONY: all run-io run-ospi-single run-ospi-burst run-ospi-open-drain \
	run-fifo syntax clean

all: run-fifo run-io run-ospi-single run-ospi-burst run-ospi-open-drain

syntax:
	$(SIM) -g2005 -s ospi_slave_top -o ospi_syntax $(RTL)

run-io:
	$(SIM) -g2005 -s tb_io_top -o simv_io \
		../rtl/io_top.v tb_io_top.v
	$(VVP) simv_io

run-fifo:
	$(SIM) -g2005 -s tb_async_fifo -o simv_fifo \
		../rtl/async_fifo.v tb_async_fifo.v
	$(VVP) simv_fifo

run-ospi-single:
	$(SIM) -g2005 -s tb_ospi_slave -o simv_ospi_single \
		$(RTL) tb_ospi_slave.v
	$(VVP) simv_ospi_single

run-ospi-burst:
	$(SIM) -g2005 -DSINGLE_TRAN=0 -s tb_ospi_slave -o simv_ospi_burst \
		$(RTL) tb_ospi_slave.v
	$(VVP) simv_ospi_burst

run-ospi-open-drain:
	$(SIM) -g2005 -DTB_SRDY_OPEN_DRAIN=1 -s tb_ospi_slave \
		-o simv_ospi_open_drain $(RTL) tb_ospi_slave.v
	$(VVP) simv_ospi_open_drain

clean:
	rm -f simv simv_fifo simv_io simv_ospi simv_ospi_single simv_ospi_burst \
		simv_ospi_open_drain ospi_syntax *.vcd *.fst *.fsdb *.log
