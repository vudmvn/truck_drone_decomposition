# Correctness-first CLI update for `vudmvn/truck_drone_decomposition`

Upstream repository: https://github.com/vudmvn/truck_drone_decomposition (branch `main`, reviewed 2026-11-07). This bundle is **not** a GitHub commit, and source has **not** been compiled with real Gurobi or OR-Tools here.

## Apply

Clone the upstream repository on a machine with network access, then:

```bash
python3 tools/apply_correctness_fixes.py /path/to/truck_drone_decomposition --check
# Option A: Export the 9 COMPLETE .cpp/.h replacements without editing the repo:
python3 tools/apply_correctness_fixes.py /path/to/truck_drone_decomposition --output-dir ./updated_source_files
# Option B: Update the checked-out repository in place (creates *.backup):
python3 tools/apply_correctness_fixes.py /path/to/truck_drone_decomposition
cd /path/to/truck_drone_decomposition
cmake -S . -B build -DBBCP_WITH_GUROBI=ON -DBBCP_WITH_ORTOOLS=ON
cmake --build build -j
./build/truck_drone_solver --list-variants
```

Running `--check` first asserts that every patch anchor matches the upstream revision; none of the source files are changed. Applied files get `.backup` copies. A later upstream commit may require re-basing the patch.

## Valid combinations

| CLI assignment | CLI slave | Model solved | Status |
|---|---|---|---|
| `explicit` | `cp` | One-drone CP for each fixed `S_k` | Supported with OR-Tools |
| `explicit` | `milp` | One-drone MILP for each fixed `S_k` | Supported with Gurobi |
| `slave-assign` | `cp` | K-drone assignment/scheduling CP | Supported with OR-Tools |
| `slave-assign` | `milp` | Existing MILP is **one-drone**, not K-drone | Rejected with clear error |

Example commands:

```bash
./build/truck_drone_solver data/instances/toy_6.txt --assignment explicit --slave milp --truck sec --time-limit 3600 --slave-time 30
./build/truck_drone_solver data/instances/toy_6.txt --assignment explicit --slave cp --truck mtz --mipnode-cuts off
./build/truck_drone_solver data/instances/toy_6.txt --assignment slave-assign --slave cp --truck sec --local-search off
```

Legacy positional arguments are preserved (after instance filename): `n_drones swap_minutes master_time slave_time master_mode result_csv branching slave_solver assignment_mode`.

## Fixes

- `main.cpp`: eliminates assignment override; adds `--help`, `--list-variants` and named flags; validates model/backend compatibility.
- `SlaveSolverFactory.cpp/.h`: checks whether backend solves correct one- or K-drone slave, no silent CP fallback.
- `SingleDroneOrderingCPSatSlave.cpp/.h`: supports `K=1` and `K=inst.n_drones`; when `0<m<K`, creates only `m` nonempty routes; maintains CP certified LB on UNKNOWN; pair energy propagation.
- `MasterCallback.cpp`: removes cutoff-driven global no-good, separates with valid certified LB; aborts unproved integer candidates; handles exceptions; correct integer cutoff from real UB.
- `MasterModelBuilder.cpp`: MTZ big-M `n+1`, initial-UB pair cut strictness.
- `MasterProblem.cpp`: passes mode to factory; disables one-drone local search on slave-assign; captures solver statuses and LB/UB into result stats.
- `Options.h`: disables UB-only global user cuts by default; excludes cutoff-specific no-good from user cuts.

## Smoke checks (without solvers)

```bash
bash tests/run_cli_mock_tests.sh
```

The tests compile the two new complete `.cpp` files against explicitly named **mock** APIs; they do not establish solver-level correctness. The installer tests all replacement anchors on the real upstream source before writing. Full source of the remaining modified `.cpp/.h` files is exported from the checked-out repository using `--output-dir`.

## Important status and proof considerations

1. `GRB_OPTIMAL` of the **relaxed Master** alone is NOT proof of optimality for the original truck-drone problem. A valid original-problem UB and a certified Master LB are required; they must coincide within tolerance.
2. A feasible but unproven CP schedule supplies an **upper bound**, not an optimality cut. `UNKNOWN` never implies infeasibility. If a Master integer solution cannot be verified or cut, this revision aborts (Gurobi `INTERRUPTED`) rather than silently accepting it.
3. Deadline-restricted CP `INFEASIBLE` means the set cannot achieve a makespan under the deadline. It does not mean that the drone set is globally infeasible.
4. All selected drone customers must be covered **exactly once** by the decoded schedule. Verify this as part of end-to-end tests before interpreting numerical results.
5. Master heuristic warm starts, UB-based preprocessing and static cuts should be tested against a small exact monolithic or enumeration baseline. A real solver build and runtime regression suite remains necessary.

## Scope

This is a correctness-focused patch. It does **not** implement a K-drone MILP or change the physical operating assumptions. DP/DSSR and more aggressive Benders cut improvements should be implemented after the three supported baselines agree on small instances.
