# CLI matrix for correctness-first update

Run `truck_drone_solver --list-variants` after compiling.

## Assignment × solver

| Assignment | `cp` | `milp` |
|---|---|---|
| `explicit` | One-drone CP-SAT solved separately for each fixed assignment `S_k` (OR-Tools required) | One-drone Gurobi MILP (supported) |
| `slave-assign` | K-drone CP-SAT selects assignment and chains (OR-Tools required) | Rejected: existing MILP only solves one-drone problem |

`--truck sec` and `--truck mtz` work with each supported row. The six branching selectors (`default`, `assignment`, `cp-aware`, `ambiguity`, `drone-benefit`, `hybrid`) remain available. `--mipnode-cuts on|off` supports ablation; use `off` for the correctness-first baseline.

## Commands

```bash
./build/truck_drone_solver --help
./build/truck_drone_solver --list-variants
./build/truck_drone_solver data/instances/toy_6.txt --assignment explicit --slave milp --truck sec --branching hybrid --time-limit 3600 --slave-time 30 --mipnode-cuts off
./build/truck_drone_solver data/instances/toy_6.txt --assignment explicit --slave cp --truck mtz --time-limit 3600 --slave-time 30
./build/truck_drone_solver data/instances/toy_6.txt --assignment slave-assign --slave cp --truck sec --time-limit 3600 --slave-time 30
```

Output lines starting `[CONFIG]`, `[SLAVE]`, `[STATUS]` show selected architecture and actual engine. Status `INTERRUPTED` means no final exactness certificate. `GRB_OPTIMAL` for the relaxed Master is not alone sufficient: check that certified LB matches valid original-problem UB.

## Safety-by-design switches

- `--assignment` is never silently overwritten.
- Incompatible assignment+backend combinations are rejected; unavailable OR-Tools is not silently redirected to one-drone MILP.
- `0<m<K` uses exactly `m` nonempty CP chains, not a fake zero makespan.
- `--local-search on` is accepted only in `explicit` mode; otherwise rejected.
- UB-restricted no-good and user-cut generation is disabled; `UNKNOWN` does not imply CP infeasibility.
- CP results carry certified lower bounds, while incumbent schedules undergo checks for customer coverage, battery feasibility, and timing.
- If the callback cannot verify a candidate or produce a violated cut, optimization is aborted instead of claiming an exact solution.

## Limitations

The release is a **source-level correction package**. The complete updated file content of the seven largest unchanged-plus-patched files is produced by `tools/apply_correctness_fixes.py --output-dir` when run against the upstream checkout. The package itself contains two complete new `.cpp` replacements and the deterministic source-to-source updater, not the entire cloned upstream repository. As there was no network access from the container or installed Gurobi/OR-Tools, no real solver execution or numerical exactness comparison is claimed.

## Lipschitz LBBD cut status

All three available assignment/solver variants use Lipschitz LBBD by default:
- `explicit+cp` and `explicit+milp`: per-drone `zK[k]` cuts.
- `slave-assign+cp`: global `zD` cuts.

There is **no** `--lbbd-cut` selector: the old uniform Big-M subset cuts are
removed. MTZ and one-drone MILP battery propagation still require their own
logic-model Big-M/indicator construction; these are **not** LBBD cuts.
