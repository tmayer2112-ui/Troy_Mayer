# Orthosis EMG decoder. `make all` regenerates every number in results/summary.md.
PY  ?= python3
RUN := PYTHONPATH=. $(PY)

.PHONY: all setup data audit test board lda cnn rate report claims clean

all: data audit test board lda cnn rate report claims

setup:
	$(PY) -m pip install -r requirements.txt

data:                  ## ~2 GB git clone of the public LibEMG MultiDay dataset
	./scripts/get_data.sh

audit:                 ## ~30 s: duplicate days, electrode polarity changes, RMS across each change
	$(RUN) scripts/audit_data.py

test:
	$(RUN) -m pytest -q tests

board:                 ## filter corners, v1 front-end gain/CMRR vs electrode impedance (no data needed)
	$(RUN) scripts/verify_board.py

lda:                   ## ~5 min on 4 cores: every protocol and both sweeps
	$(RUN) scripts/run_lda.py 4

cnn:                   ## ~25 min on 4 CPU cores (17 of it training): 8 variants x 3 seeds, plus the matched LDA control
	$(RUN) scripts/run_cnn.py 4
	$(RUN) scripts/lda_control.py

rate:                  ## <1 min: v2 sampled at the firmware's 1 kHz instead of 2048 Hz
	$(RUN) scripts/sample_rate.py 3

report:
	$(RUN) scripts/report.py

claims:                ## every portfolio and resume number vs the file that produced it
	$(RUN) scripts/check_claims.py

clean:
	rm -f data/multiday_cache.npz
