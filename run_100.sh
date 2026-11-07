#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Configuration
# ============================================================

EXE="./build/truck_drone_solver"
DATA_DIR="./data/instances"

# Runtime libraries
export LD_LIBRARY_PATH=/opt/or-tools_x86_64_Debian-11_cpp_v9.9.3963/lib:/home/orlab/gurobi1301/linux64/lib:${LD_LIBRARY_PATH:-}

# Solver parameters
TARGET_N_CUSTOMERS=30
N_DRONES=4
MASTER_TIME=3600
CP_TIME=5

# Master settings
# Options:
#   SUBTOUR_MODE: mtz | sec | cut | lazy | lazycut
#   BRANCHING: default | assignment-first | cp-aware | ambiguity-first | drone-benefit | hybrid
#   SLAVE_TYPE: cp | milp
#   ASSIGNMENT_MODE: explicit | slave-assign
SUBTOUR_MODE="sec"
BRANCHING="hybrid"
SLAVE_TYPE="cp"
ASSIGNMENT_MODE="explicit"

# Output files
RESULT_DIR="./results"
LOG_DIR="./logs_${TARGET_N_CUSTOMERS}_${SUBTOUR_MODE}_${BRANCHING}_${SLAVE_TYPE}_${ASSIGNMENT_MODE}"

RESULT_CSV="${RESULT_DIR}/results_${TARGET_N_CUSTOMERS}_${SUBTOUR_MODE}_${BRANCHING}_${SLAVE_TYPE}_${ASSIGNMENT_MODE}.csv"

mkdir -p "$RESULT_DIR"
mkdir -p "$LOG_DIR"

# Overwrite old logs.
rm -f "$LOG_DIR"/*.txt

# If you want to restart CSV from scratch, uncomment this line.
# rm -f "$RESULT_CSV"

# ============================================================
# Check executable
# ============================================================

if [[ ! -x "$EXE" ]]; then
    echo "[ERROR] Executable not found or not executable: $EXE"
    echo "Build first:"
    echo "  rm -rf build"
    echo "  mkdir build"
    echo "  cd build"
    echo "  cmake .. \\"
    echo "    -DGUROBI_HOME=/home/orlab/gurobi1301/linux64 \\"
    echo "    -DORTOOLS_HOME=/opt/or-tools_x86_64_Debian-11_cpp_v9.9.3963 \\"
    echo "    -DCMAKE_PREFIX_PATH=/opt/or-tools_x86_64_Debian-11_cpp_v9.9.3963"
    echo "  cmake --build . -j"
    exit 1
fi

# ============================================================
# Check data directory
# ============================================================

if [[ ! -d "$DATA_DIR" ]]; then
    echo "[ERROR] Data directory not found: $DATA_DIR"
    exit 1
fi

# ============================================================
# Run selected instances
# ============================================================

count=0

for file in "$DATA_DIR"/*.txt; do
    [[ -f "$file" ]] || continue

    base=$(basename "$file" .txt)

    # ------------------------------------------------------------
    # Parse filename.
    #
    # Accepted examples:
    #   30-c-1-c-swap-0.txt
    #   30-rc-1-r-swap-0.txt
    #   30-anything-swap-3.txt
    #
    # Meaning:
    #   first number      -> number of customers
    #   middle part       -> instance tag
    #   number after swap -> swap time in minutes
    # ------------------------------------------------------------
    if [[ "$base" =~ ^([0-9]+)-(.+)-swap-([0-9]+)$ ]]; then
        N_CUSTOMERS="${BASH_REMATCH[1]}"
        INSTANCE_TAG="${BASH_REMATCH[2]}"
        SWAP_MINUTES="${BASH_REMATCH[3]}"
    else
        echo "[WARNING] Filename does not match expected pattern: $base"
        echo "[WARNING] Expected: <n>-<anything>-swap-<swap_minutes>.txt"
        echo "[WARNING] Skip this file."
        continue
    fi

    if [[ "$N_CUSTOMERS" -ne "$TARGET_N_CUSTOMERS" ]]; then
        continue
    fi

    log_file="$LOG_DIR/${base}_${SUBTOUR_MODE}_${BRANCHING}_${SLAVE_TYPE}_${ASSIGNMENT_MODE}.txt"

    echo "============================================================"
    echo "[RUN]             $file"
    echo "[N]               $N_CUSTOMERS"
    echo "[TAG]             $INSTANCE_TAG"
    echo "[DRONES]          $N_DRONES"
    echo "[SWAP]            $SWAP_MINUTES minute(s)"
    echo "[MASTER TIME]     $MASTER_TIME second(s)"
    echo "[CP TIME]         $CP_TIME second(s)"
    echo "[SUBTOUR MODE]    $SUBTOUR_MODE"
    echo "[BRANCHING]       $BRANCHING"
    echo "[SLAVE TYPE]      $SLAVE_TYPE"
    echo "[ASSIGNMENT MODE] $ASSIGNMENT_MODE"
    echo "[CSV]             $RESULT_CSV"
    echo "[LOG]             $log_file"
    echo "============================================================"

    "$EXE" \
        "$file" \
        "$N_DRONES" \
        "$SWAP_MINUTES" \
        "$MASTER_TIME" \
        "$CP_TIME" \
        "$SUBTOUR_MODE" \
        "$RESULT_CSV" \
        "$BRANCHING" \
        "$SLAVE_TYPE" \
        "$ASSIGNMENT_MODE" \
        2>&1 | tee "$log_file"

    count=$((count + 1))

    echo
    echo "[DONE INSTANCE] $file"
    echo
done

echo "============================================================"
echo "[ALL DONE]"
echo "Finished $count instance(s) with $TARGET_N_CUSTOMERS customers."
echo "[CSV] $RESULT_CSV"
echo "[LOG] $LOG_DIR"
echo "============================================================"