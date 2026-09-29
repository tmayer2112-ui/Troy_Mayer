# Orthosis EMG decoder. `make all` regenerates every number in results/summary.md.
PY  ?= python3
RUN := PYTHONPATH=. $(PY)

.PHONY: all setup data audit test board lda cnn report clean

all: data audit test board lda cnn report

setup:
	$(PY) -m pip install -r requirements.txt

data:                  ## ~2 GB git clone of the public LibEMG MultiDay dataset
	./scripts/get_data.sh

audit:                 ## duplicate days and the day-68 polarity flip
	$(RUN) scripts/audit_data.py

test:
	$(RUN) -m pytest -q tests

board:                 ## filter corners, v1 front-end gain/CMRR vs electrode impedance (no data needed)
	$(RUN) scripts/verify_board.py

lda:                   ## ~20 min on 4 cores: every protocol and both sweeps
	$(RUN) scripts/run_lda.py 4

cnn:                   ## ~25 min on 4 CPU cores: 8 variants x 3 seeds, plus the matched LDA control
	$(RUN) scripts/run_cnn.py 4
	$(RUN) scripts/lda_control.py

report:
	$(RUN) scripts/report.py

clean:
	rm -f data/multiday_cache.npz
