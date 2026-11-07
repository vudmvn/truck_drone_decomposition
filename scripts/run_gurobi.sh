#!/usr/bin/env bash
set -euo pipefail

INSTANCE="${1:-data/instances/legacy_toy_6.txt}"
GUROBI_LIB="${GUROBI_LIB:-gurobi120}"

mkdir -p build

cmake -S . -B build \
  -DBBCP_WITH_GUROBI=ON \
  -DBBCP_WITH_ORTOOLS=ON \
  -DGUROBI_MAIN_LIB="${GUROBI_LIB}"

cmake --build build -j

./build/truck_drone_solver "$INSTANCE" 2 3 3600 5 cut results/csv/result_gurobi.csv hybrid cp
