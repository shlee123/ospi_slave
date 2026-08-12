SIM ?= iverilog
TOP ?= tb_async_fifo

RTL := ../rtl/async_fifo.v
TB  := tb_async_fifo.v

.PHONY: all run clean

all: run

run:
	$(SIM) -g2005 -s $(TOP) -o simv $(RTL) $(TB)
	vvp simv

clean:
	rm -f simv *.vcd *.fst *.fsdb *.log
