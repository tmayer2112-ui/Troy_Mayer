#!/usr/bin/env bash
# GitHub Pages on this repo publishes from /docs, so docs/ must mirror the
# site at the repo root. Run this after editing index.html, then commit both.
set -euo pipefail
cd "$(dirname "$0")"
rm -rf docs && mkdir -p docs
cp index.html docs/
# copy every local asset index.html references -- src=, href=, and CSS url()
{ grep -oE '(src|href)="[^"#][^"]*"' index.html | sed -E 's/.*="//; s/"//'
  grep -oE "url\(['\"]?[^)'\"]+['\"]?\)" index.html | sed -E "s/url\(['\"]?//; s/['\"]?\)//"
} | grep -vE '^(https?:|mailto:|data:)' | sort -u \
  | while read -r f; do [ -f "$f" ] && mkdir -p "docs/$(dirname "$f")" && cp "$f" "docs/$f"; done  # keep paths
touch docs/.nojekyll   # serve files as-is, skip Jekyll
echo "docs/ synced: $(ls -A docs | wc -l) files"
