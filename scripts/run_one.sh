#!/usr/bin/env bash
set -euo pipefail

GUROBI_LIB="${GUROBI_LIB:-gurobi120}"

mkdir -p build

cmake -S . -B build \
  -DBBCP_WITH_GUROBI=ON \
  -DBBCP_WITH_ORTOOLS=ON \
  -DGUROBI_MAIN_LIB="${GUROBI_LIB}"

cmake --build build -j

./build/truck_drone_solver data/instances/legacy_toy_6.txt 2 3 3600 5 cut results/csv/result.csv hybrid cp
