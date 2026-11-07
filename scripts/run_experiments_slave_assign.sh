#!/usr/bin/env bash
# ==============================================================================
# Batch Experiment Runner for Truck-Drone Decomposition Solver
#
# Configuration:
#   Master: MILP (Gurobi 13.0.1) with SEC lazy constraints
#   Slave: CP-SAT (OR-Tools 9.9)
#   Assignment: slave-assign
#   Branching: hybrid
#   Time limit: up to 3600s per instance
#   Sizes: n = 30, 50, 100 (Total 243 instances)
#   Storage: CSV & SQLite database (results/results.db)
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${BASE_DIR}"

# Build solver if not already present
if [ ! -f "build/truck_drone_solver" ]; then
    echo "[BUILD] Compiling truck_drone_solver..."
    mkdir -p build
    cmake -S . -B build -DBBCP_WITH_GUROBI=ON -DBBCP_WITH_ORTOOLS=ON -DGUROBI_MAIN_LIB=gurobi130
    cmake --build build -j
fi

export PYTHONUNBUFFERED=1

python3 "${SCRIPT_DIR}/run_experiments_slave_assign.py" "$@"
