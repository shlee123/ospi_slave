SIM ?= iverilog
VVP ?= vvp
# AXI_WIDTHS rule: exhaustive supported AXI_DATA_WIDTH power-of-two values.
AXI_WIDTHS ?= 8 16 32 64 128 256 512 1024
RTL := ../rtl/async_fifo.v ../rtl/io_top.v ../rtl/ospi_slave.v \
	../rtl/ospi_slave_top.v

.PHONY: all run-io run-ospi-single run-ospi-burst run-ospi-open-drain \
	run-width-single run-width-burst-4k run-max-burst run-fifo syntax clean

all: run-fifo run-io run-ospi-single run-ospi-burst run-ospi-open-drain \
	run-width-single run-width-burst-4k run-max-burst

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

# SINGLE_TRAN=1 regression: every supported width must use AxLEN=0 and
# serialize a 32-bit OSPI word into multiple transactions on 8/16-bit buses.
run-width-single:
	set -e; for width in $(AXI_WIDTHS); do \
		$(SIM) -g2005 -DTB_AXI_DATA_WIDTH=$$width \
			-s tb_ospi_slave -o simv_width_single $(RTL) tb_ospi_slave.v; \
		$(VVP) simv_width_single; \
	done

# SINGLE_TRAN=0 regression: start the 8-byte request at 0xFFC. The testbench
# requires both Write and Read to split at the 4KB boundary for every width.
run-width-burst-4k:
	set -e; for width in $(AXI_WIDTHS); do \
		$(SIM) -g2005 -DSINGLE_TRAN=0 -DTB_AXI_DATA_WIDTH=$$width \
			-DTB_TEST_ADDRESS=4092 -DTB_4K_BOUNDARY=1 \
			-s tb_ospi_slave -o simv_width_burst_4k $(RTL) tb_ospi_slave.v; \
		$(VVP) simv_width_burst_4k; \
	done

# Maximum burst regression: 512 bytes on AXI_DATA_WIDTH=8 creates 512 beats,
# which must be split into two transactions with 256 beats each.
run-max-burst:
	$(SIM) -g2005 -DSINGLE_TRAN=0 -DTB_AXI_DATA_WIDTH=8 \
		-DTB_TEST_ADDRESS=512 -DTB_LONG_BURST=1 \
		-s tb_ospi_slave -o simv_max_burst $(RTL) tb_ospi_slave.v
	$(VVP) simv_max_burst

clean:
	rm -f simv simv_fifo simv_io simv_ospi simv_ospi_single simv_ospi_burst \
		simv_ospi_open_drain simv_width_single simv_width_burst_4k \
		simv_max_burst ospi_syntax *.vcd *.fst *.fsdb *.log
