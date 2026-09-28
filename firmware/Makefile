# Orthosis firmware: host tests, simulation results and figures.
#   make test      unit + closed-loop tests (the controller against the motor/limb model)
#   make results   traces, metrics.json and README figures
BUILD ?= $(CURDIR)/build
PY    ?= python3

.PHONY: all build test latency results clean
all: test results

build:
	cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD) -j4

test: build
	cd $(BUILD) && ctest --output-on-failure
	$(BUILD)/sketch_syntax_check > /dev/null && echo "sketch compiles and setup() runs"

latency: build         ## end-to-end latency sweep with real EMG (sim/data/real_contractions.bin)
	$(BUILD)/latency_sweep sim/data/real_contractions.bin

results: build
	mkdir -p results/figures
	$(BUILD)/sim_demo results
	$(PY) sim/plot.py results

clean:
	rm -rf $(BUILD)
