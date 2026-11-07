#!/usr/bin/env bash
set -euo pipefail
mkdir -p build
cmake -S . -B build
cmake --build build -j
for f in data/instances/*.txt; do
  ./build/truck_drone_solver "$f" single-drone-ordering results/csv/batch.csv | tee "results/logs/$(basename "$f").log"
done
