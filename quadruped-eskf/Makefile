# Quadruped contact-aided ESKF. `make all` regenerates every number in README.md from scratch.
PY      ?= python3
RUN     := PYTHONPATH=python $(PY)
BUILD   := build

.PHONY: all setup build test test-cpp test-py data tune reproduce observability experiments timing claims clean

all: build test data tune reproduce observability experiments timing claims

setup:
	$(PY) -m pip install -r requirements.txt

build:
	cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD) -j

test: test-cpp test-py

test-cpp: build
	cd $(BUILD) && ctest --output-on-failure

test-py:
	$(RUN) -m pytest -q python/tests

data:                     ## ~2 min: headline walk x 10 noise seeds, tuning walk, 300 s long walk
	$(RUN) python/scripts/generate_data.py

tune: build               ## ~5 min: contact-model grid on the TUNING walk -> results/tuned_params.json
	$(RUN) python/scripts/tune.py

reproduce: build          ## ~3 min: headline numbers, RMSE table, figures, results/logs/reproduce.log
	$(RUN) python/scripts/reproduce.py

observability: build      ## covariance split over 300 s + null-space check
	$(RUN) python/scripts/observability.py

experiments: build        ## ~15 min: hard/soft, slip, friction, contact delay, process noise
	$(RUN) python/scripts/experiments.py

timing: build             ## 60 s wall clock; run on an idle machine
	$(RUN) python/scripts/timing.py

claims:                   ## every resume number vs the file that produced it
	$(RUN) python/scripts/check_claims.py

clean:
	rm -rf $(BUILD) data/cpp_run data/timing
