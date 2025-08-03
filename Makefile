
# Makefile for MODBUS Smart Vending Machine
# Supports multiple FPGA vendors and simulators

# Tool selection
SIMULATOR ?= iverilog
FPGA_VENDOR ?= xilinx

# Source files
SOURCES = uart_interface.v dual_clock_fifo.v modbus_fsm.v dual_port_ram.v modbus_smart_vending_machine.v
TESTBENCH = modbus_vending_tb.v
TOP_MODULE = modbus_smart_vending_machine
TB_MODULE = modbus_vending_tb

# Simulation targets
sim: $(SOURCES) $(TESTBENCH)
	@echo "Running simulation with $(SIMULATOR)..."
ifeq ($(SIMULATOR),iverilog)
	iverilog -o sim.out $(SOURCES) $(TESTBENCH)
	vvp sim.out
	@echo "Simulation completed. Check modbus_vending_tb.vcd for waveforms"
else ifeq ($(SIMULATOR),modelsim)
	vlog $(SOURCES) $(TESTBENCH)
	vsim -c -do "run -all; quit" $(TB_MODULE)
else
	@echo "Unsupported simulator: $(SIMULATOR)"
	@exit 1
endif

# Synthesis targets
synth:
ifeq ($(FPGA_VENDOR),xilinx)
	@echo "Running Xilinx Vivado synthesis..."
	vivado -mode batch -source synthesis_script.tcl
else ifeq ($(FPGA_VENDOR),intel)
	@echo "Running Intel Quartus synthesis..."
	quartus_sh --flow compile modbus_vending
else
	@echo "Unsupported FPGA vendor: $(FPGA_VENDOR)"
	@exit 1
endif

# Clean targets
clean:
	rm -rf *.out *.vcd *.wlf work/ transcript
	rm -rf modbus_vending_project/
	rm -rf db/ incremental_db/ output_files/

# Lint checking
lint:
	verilator --lint-only --top-module $(TOP_MODULE) $(SOURCES)

# Documentation
docs:
	@echo "Generating documentation..."
	@echo "MODBUS Smart Vending Machine - File List:" > README.md
	@echo "===========================================" >> README.md
	@for file in $(SOURCES) $(TESTBENCH); do echo "- $$file"; done >> README.md

# Help
help:
	@echo "Available targets:"
	@echo "  sim     - Run simulation (default: iverilog)"
	@echo "  synth   - Run synthesis (default: xilinx)"  
	@echo "  lint    - Run lint checking with Verilator"
	@echo "  clean   - Clean generated files"
	@echo "  docs    - Generate documentation"
	@echo "  help    - Show this help"
	@echo ""
	@echo "Variables:"
	@echo "  SIMULATOR   - iverilog, modelsim (default: iverilog)"
	@echo "  FPGA_VENDOR - xilinx, intel (default: xilinx)"

.PHONY: sim synth clean lint docs help
