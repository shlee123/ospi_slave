SHELL := /bin/bash
.SHELLFLAGS := -o pipefail -ec

# SIMULATOR options: vcs (default) or iverilog.
SIMULATOR ?= vcs
# TESTBENCH rule: Verilog testbench source relative to sim/ (or an absolute path).
TESTBENCH ?= tb_ospi_slave.v
# DEFINES rule: space-separated preprocessor definitions, for example
# DEFINES="SINGLE_TRAN=0 TB_AXI_DATA_WIDTH=64".
DEFINES ?=

# Debug rule:
#   1 = terminate the testbench immediately on the first reported regression error.
#   0 = accumulate regression errors and terminate after the test completes.
STOP_ON_ERROR ?= 1

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
# RUN_NAME rule: append sorted NAME-VALUE compile definitions so parameterized
# runs keep independent executables, waveforms and logs.
empty :=
space := $(empty) $(empty)
ALL_DEFINES := TB_STOP_ON_ERROR=$(STOP_ON_ERROR) $(strip $(DEFINES))
SORTED_DEFINES := $(sort $(strip $(ALL_DEFINES)))
DEFINE_TAG := $(subst $(space),__,$(subst =,-,$(SORTED_DEFINES)))
RUN_NAME ?= $(TB_TOP)$(if $(DEFINE_TAG),__$(DEFINE_TAG))

BUILD_DIR := $(BUILD_ROOT)/$(SIMULATOR)/$(RUN_NAME)
SIMV := $(BUILD_DIR)/simv
FSDB_FILE := $(FSDB_DIR)/$(RUN_NAME).fsdb
LOG_FILE := $(LOG_DIR)/$(RUN_NAME).log

# Tool option variables may be extended or overridden by the local environment.
IVERILOG_FLAGS ?= -g2005
VCS_FLAGS ?= -full64 -sverilog -debug_access+all -kdb
VERDI_FLAGS ?=
# auto probes this Verdi release; use none to omit the language option or
# provide an explicit option (for example, -sv) for a site-specific setup.
VERDI_LANGUAGE_FLAG ?= auto
FSDB_PLI_DIR ?= $(VERDI_HOME)/share/PLI/VCS/LINUX64
FSDB_PLI ?= -P $(FSDB_PLI_DIR)/novas.tab $(FSDB_PLI_DIR)/pli.a

IVERILOG_DEFINES := $(addprefix -D,$(ALL_DEFINES))
VCS_DEFINES := $(addprefix +define+,$(ALL_DEFINES))

.PHONY: all sim compile run verdi help dirs check-testbench check-tools \
	run-io run-ospi-single run-ospi-burst run-ospi-open-drain \
	run-width-single run-width-burst-4k run-max-burst run-fifo \
	run-apb-ospi syntax clean

all: run-fifo run-io run-apb-ospi run-ospi-single run-ospi-burst run-ospi-open-drain \
	run-width-single run-width-burst-4k run-max-burst

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

verdi: dirs check-testbench
	@command -v "$(VERDI)" >/dev/null || { \
		echo "ERROR: Verdi executable not found: $(VERDI)" >&2; exit 2; \
	}
	@test -f "$(FSDB_FILE)" || { \
		echo "ERROR: FSDB not found: $(FSDB_FILE); run with SIMULATOR=vcs first." >&2; \
		exit 2; \
	}
	@verdi_language_flag="$(VERDI_LANGUAGE_FLAG)"; \
	if [[ "$$verdi_language_flag" == "auto" ]]; then \
		verdi_help="$$("$(VERDI)" -help 2>&1 || true)"; \
		if grep -Eq -- '(^|[[:space:]])-sv([[:space:],]|$$)' <<<"$$verdi_help"; then \
			verdi_language_flag="-sv"; \
		elif grep -Eq -- '(^|[[:space:]])-v2k([[:space:],]|$$)' <<<"$$verdi_help"; then \
			verdi_language_flag="-v2k"; \
		else \
			verdi_language_flag=""; \
		fi; \
	elif [[ "$$verdi_language_flag" == "none" ]]; then \
		verdi_language_flag=""; \
	fi; \
	echo "INFO: Verdi language option: $${verdi_language_flag:-<none>}"; \
	"$(VERDI)" $$verdi_language_flag -f "$(FILELIST)" "$(TESTBENCH)" \
		-top "$(TB_TOP)" $(VCS_DEFINES) -ssf "$(FSDB_FILE)" \
		$(VERDI_FLAGS) &

syntax: dirs
	@$(IVERILOG) $(IVERILOG_FLAGS) -s ospi_slave_top -f "$(FILELIST)" \
		-o "$(BUILD_DIR)/ospi_syntax" 2>&1 | tee "$(LOG_DIR)/syntax.log"

run-io:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_io_top.v

run-fifo:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_async_fifo.v

run-apb-ospi:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_apb_ospi.v

run-ospi-single:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v

run-ospi-burst:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
		DEFINES="SINGLE_TRAN=0"

run-ospi-open-drain:
	@$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
		DEFINES="TB_SRDY_OPEN_DRAIN=1"

run-width-single:
	@for width in $(AXI_WIDTHS); do \
		$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
			DEFINES="TB_AXI_DATA_WIDTH=$$width"; \
	done

run-width-burst-4k:
	@for width in $(AXI_WIDTHS); do \
		$(MAKE) --no-print-directory sim TESTBENCH=tb_ospi_slave.v \
			DEFINES="SINGLE_TRAN=0 TB_AXI_DATA_WIDTH=$$width \
			TB_TEST_ADDRESS=4092 TB_4K_BOUNDARY=1"; \
	done

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
	@echo "  verdi    Open sources and the FSDB matching TESTBENCH plus DEFINES."
	@echo "  all      Run the complete regression suite."
	@echo "  run-apb-ospi  Run Program/Direct APB-to-OSPI-to-AXI verification."
	@echo "  syntax   Run an Icarus RTL syntax check."
	@echo "  clean    Remove generated build, FSDB and log outputs."
	@echo "  help     Show this help text."
	@echo
	@echo "Variables:"
	@echo "  SIMULATOR=vcs|iverilog  Simulation tool (default: vcs)."
	@echo "  TESTBENCH=<file.v>      Testbench source (default: tb_ospi_slave.v)."
	@echo "  STOP_ON_ERROR=1|0       1=finish immediately, 0=accumulate errors (default: 1)."
	@echo "  DEFINES=\"NAME=VALUE ...\" Verilog preprocessor definitions."
	@echo "  AXI_WIDTHS=\"8 16 ...\" Width list passed as TB_AXI_DATA_WIDTH per run."
	@echo "  RUN_NAME=<name>         Override generated testbench/parameter output name."
	@echo "  VERDI_HOME=<path>       Verdi installation used for VCS FSDB PLI."
	@echo "  VERDI_LANGUAGE_FLAG=auto|none|<flag>  Language option (default: auto)."
	@echo
	@echo "Examples:"
	@echo "  make run-apb-ospi SIMULATOR=iverilog"
	@echo "  make run-apb-ospi SIMULATOR=vcs STOP_ON_ERROR=0"
	@echo "  make sim TESTBENCH=tb_async_fifo.v"
	@echo "  make sim DEFINES=\"TB_AXI_DATA_WIDTH=64 SINGLE_TRAN=0\""
	@echo "  make verdi TESTBENCH=tb_apb_ospi.v"

clean:
	@rm -rf "$(BUILD_ROOT)" "$(FSDB_DIR)" "$(LOG_DIR)"
