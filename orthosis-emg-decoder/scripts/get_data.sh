#!/usr/bin/env bash
# Kaufmann, Englehart & Platzner, "Fluctuating EMG signals: investigating long-term
# effects of pattern matching algorithms", IEEE EMBC 2010. Mirrored by LibEMG.
set -euo pipefail
dest="${MULTIDAY_DIR:-data/multiday}"
if [ -d "$dest" ] && ls "$dest"/S0_D1_C0.csv >/dev/null 2>&1; then
  echo "$dest already present"; exit 0
fi
mkdir -p "$(dirname "$dest")"
git clone --depth 1 https://github.com/LibEMG/MultiDay "$dest"
