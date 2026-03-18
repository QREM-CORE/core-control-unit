# Default simulation tool
SIM ?= verilator

.PHONY: all run_all clean

all: run_all

run_all:
	@echo "Target: run_all executed."
	@echo "Simulation tool selected: $(SIM)"
	@echo "Note: Testbenches are not yet implemented. Exiting cleanly for CI."
	@exit 0

clean:
	@echo "Cleaning workspace..."
	rm -rf obj_dir transcript work *.log *.vcd