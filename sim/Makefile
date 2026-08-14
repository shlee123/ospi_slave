SHELL := /bin/bash
.SHELLFLAGS := -o pipefail -ec

# SIMULATOR options: iverilog (default) or vcs.
SIMULATOR ?= iverilog
# TESTBENCH rule: Verilog testbench source relative to sim/ (or an absolute path).
TESTBENCH ?= tb_ospi_slave.v
# DEFINES rule: space-separated preprocessor definitions, for example
# DEFINES="SINGLE_TRAN=0 TB_AXI_DATA_WIDTH=64".
DEFINES ?=

IVERILOG ?= iverilog
VVP ?= vvp
VCS ?= vcs
VERDI ?= verdi
VERDI_HOME ?= $(NOVAS_HOME)

# AXI_WIDTHS rule: exhaustive supported AXI_DATA_WIDTH power-of-two values.
AXI_WIDTHS ?= 8 16 32 64 128 256 512 1024
FILELIST ?= filelist.f
BUILD_ROOT ?= build
FSDB_DIR ?= fsdb
LOG_DIR ?= log

TB_TOP := $(basename $(notdir $(TESTBENCH)))
BUILD_DIR := $(BUILD_ROOT)/$(SIMULATOR)/$(TB_TOP)
SIMV := $(BUILD_DIR)/simv
FSDB_FILE := $(FSDB_DIR)/$(TB_TOP).fsdb
LOG_FILE := $(LOG_DIR)/$(TB_TOP).log

# Tool option variables may be extended or overridden by the local environment.
IVERILOG_FLAGS ?= -g2005
VCS_FLAGS ?= -full64 -v2005 -debug_access+all -kdb
VERDI_FLAGS ?=
FSDB_PLI_DIR ?= $(VERDI_HOME)/share/PLI/VCS/LINUX64
FSDB_PLI ?= -P $(FSDB_PLI_DIR)/novas.tab $(FSDB_PLI_DIR)/pli.a

IVERILOG_DEFINES := $(addprefix -D,$(DEFINES))
VCS_DEFINES := $(addprefix +define+,$(DEFINES))

.PHONY: all sim compile run verdi help dirs check-testbench check-tools \
	run-io run-ospi-single run-ospi-burst run-ospi-open-drain \
	run-width-single run-width-burst-4k run-max-burst run-fifo syntax clean

all: run-fifo run-io run-ospi-single run-ospi-burst run-ospi-open-drain \
	run-width-single run-width-burst-4k run-max-burst

# The requested output directories are checked and created before simulation.
dirs:
	@mkdir -p "$(BUILD_DIR)" "$(FSDB_DIR)" "$(LOG_DIR)"

check-testbench:
	@test -f "$(TESTBENCH)" || { \
		echo "ERROR: testbench not found: $(TESTBENCH)" >&2; exit 2; \
	}

check-tools:
	@case "$(SIMULATOR)" in \
		iverilog) command -v "$(IVERILOG)" >/dev/null; \
			command -v "$(VVP)" >/dev/null ;; \
		vcs) command -v "$(VCS)" >/dev/null; \
			test -n "$(VERDI_HOME)" || { \
				echo "ERROR: set VERDI_HOME (or NOVAS_HOME) for FSDB PLI." >&2; \
				exit 2; \
			}; \
			test -f "$(FSDB_PLI_DIR)/novas.tab" || { \
				echo "ERROR: novas.tab not found under $(FSDB_PLI_DIR)" >&2; \
				exit 2; \
			}; \
			test -f "$(FSDB_PLI_DIR)/pli.a" || { \
				echo "ERROR: pli.a not found under $(FSDB_PLI_DIR)" >&2; \
				exit 2; \
			} ;; \
		*) echo "ERROR: unsupported SIMULATOR=$(SIMULATOR); use iverilog or vcs." >&2; \
			exit 2 ;; \
	esac

sim: compile
	@$(MAKE) --no-print-directory run

compile: dirs check-testbench check-tools
	@rm -f "$(LOG_FILE)"
	@if [[ "$(SIMULATOR)" == "iverilog" ]]; then \
		$(IVERILOG) $(IVERILOG_FLAGS) $(IVERILOG_DEFINES) \
			-s "$(TB_TOP)" -f "$(FILELIST)" "$(TESTBENCH)" \
			-o "$(SIMV)" 2>&1 | tee "$(LOG_FILE)"; \
	else \
		$(VCS) $(VCS_FLAGS) $(VCS_DEFINES) \
			+define+FSDB +define+FSDB_FILE=\"$(abspath $(FSDB_FILE))\" \
			-top "$(TB_TOP)" -f "$(FILELIST)" "$(TESTBENCH)" \
			-o "$(SIMV)" -Mdir="$(BUILD_DIR)/csrc" $(FSDB_PLI) \
			2>&1 | tee "$(LOG_FILE)"; \
	fi

run: dirs check-testbench check-tools
	@test -x "$(SIMV)" || { \
		echo "ERROR: simulation executable not found; run 'make compile' first." >&2; \
		exit 2; \
	}
	@if [[ "$(SIMULATOR)" == "iverilog" ]]; then \
		$(VVP) "$(SIMV)" 2>&1 | tee -a "$(LOG_FILE)"; \
	else \
		"$(SIMV)" +fsdb+autoflush 2>&1 | tee -a "$(LOG_FILE)"; \
	fi

# Open the selected testbench source environment and its matching FSDB.
verdi: dirs check-testbench
	@command -v "$(VERDI)" >/dev/null || { \
		echo "ERROR: Verdi executable not found: $(VERDI)" >&2; exit 2; \
	}
	@test -f "$(FSDB_FILE)" || { \
		echo "ERROR: FSDB not found: $(FSDB_FILE); run with SIMULATOR=vcs first." >&2; \
		exit 2; \
	}
	@$(VERDI) -v2k -f "$(FILELIST)" "$(TESTBENCH)" -top "$(TB_TOP)" \
		$(VCS_DEFINES) -ssf "$(FSDB_FILE)" $(VERDI_FLAGS) &

syntax: dirs
	@$(IVERILOG) $(IVERILOG_FLAGS) -s ospi_slave_top -f "$(FILELIST)" \
		-o "$(BUILD_DIR)/ospi_syntax" 2>&1 | tee "$(LOG_DIR)/syntax.log"

run-io:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_io_top.v

run-fifo:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_async_fifo.v

run-ospi-single:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v

run-ospi-burst:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
		DEFINES="SINGLE_TRAN=0"

run-ospi-open-drain:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
		DEFINES="TB_SRDY_OPEN_DRAIN=1"

# SINGLE_TRAN=1 regression: every supported width must use AxLEN=0 and
# serialize a 32-bit OSPI word into multiple transactions on 8/16-bit buses.
run-width-single:
	@for width in $(AXI_WIDTHS); do \
		$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
			DEFINES="TB_AXI_DATA_WIDTH=$$width"; \
	done

# SINGLE_TRAN=0 regression: start the 8-byte request at 0xFFC. The testbench
# requires both Write and Read to split at the 4KB boundary for every width.
run-width-burst-4k:
	@for width in $(AXI_WIDTHS); do \
		$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
			DEFINES="SINGLE_TRAN=0 TB_AXI_DATA_WIDTH=$$width \
			TB_TEST_ADDRESS=4092 TB_4K_BOUNDARY=1"; \
	done

# Maximum burst regression: 512 bytes on AXI_DATA_WIDTH=8 creates 512 beats,
# which must be split into two transactions with 256 beats each.
run-max-burst:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
		DEFINES="SINGLE_TRAN=0 TB_AXI_DATA_WIDTH=8 \
		TB_TEST_ADDRESS=512 TB_LONG_BURST=1"

help:
	@echo "OSPI slave simulation Makefile"
	@echo
	@echo "Targets:"
	@echo "  sim      Compile and run TESTBENCH with SIMULATOR."
	@echo "  compile  Compile TESTBENCH only."
	@echo "  run      Run the previously compiled TESTBENCH."
	@echo "  verdi    Open sources and fsdb/<testbench-name>.fsdb in Verdi."
	@echo "  all      Run the complete regression suite."
	@echo "  syntax   Run an Icarus RTL syntax check."
	@echo "  clean    Remove generated build, FSDB and log outputs."
	@echo "  help     Show this help text."
	@echo
	@echo "Variables:"
	@echo "  SIMULATOR=iverilog|vcs  Simulation tool (default: iverilog)."
	@echo "  TESTBENCH=<file.v>      Testbench source (default: tb_ospi_slave.v)."
	@echo "  DEFINES=\"NAME=VALUE ...\" Verilog preprocessor definitions."
	@echo "  VERDI_HOME=<path>       Verdi installation used for VCS FSDB PLI."
	@echo
	@echo "Examples:"
	@echo "  make sim TESTBENCH=tb_async_fifo.v"
	@echo "  make sim SIMULATOR=vcs TESTBENCH=tb_ospi_slave.v"
	@echo "  make verdi TESTBENCH=tb_ospi_slave.v"

clean:
	@rm -rf "$(BUILD_ROOT)" "$(FSDB_DIR)" "$(LOG_DIR)"
