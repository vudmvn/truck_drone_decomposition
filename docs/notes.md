# Notes and invariants

This project implements the decomposition in `docs/model_master_slave.tex`.

## Master

Master decides:

- `t_i`: customer `i` served by truck;
- `a_i_k`: customer `i` served by drone `k`;
- `x_i_j`: truck travels directly from `i` to `j`;
- `z_T`: truck route duration;
- `z_k`: lower-bound/master estimate for drone `k`;
- `z_D`: drone makespan;
- `z`: total makespan.

## Slave

For each fixed drone `k`, slave receives:

```text
S_k = { i : a_i_k = 1 }
```

and only decides:

- order of customers in `S_k`;
- battery-swap positions.

## Cut

If slave proves `D_k*(S_k)=theta`, add:

```text
z_k + M ( |S_k| - sum_{i in S_k} a_i_k ) >= theta
```

If slave proves infeasible under `UB-1`, add the same cut with `rhs = UB`.

## Critical invariant

Never update the original-problem upper bound using the master objective alone.

Only update UB by:

```text
accepted_z = max(truck_value, max_k slave_value_k)
```

after all relevant slave schedules are feasible.


## Exact Gurobi implementation added

When `BBCP_WITH_GUROBI=ON`:

1. `MasterProblem` builds the exact master model.
2. `MasterModelBuilder` creates \(t_i,a_{ik},x_{ij},q_k,z_T,z_k,z_D,z\).
3. `MasterCallback` checks each MIPSOL by calling `SingleDroneOrderingSlave` for each drone.
4. `MasterCuts` supplies the conditional cut structure.
5. `MasterProblem` exports the best valid checked solution.



## Data reader

The legacy coordinate-based reader has been restored in:

```text
include/bbcp/io/DataReader.h
src/io/DataReader.cpp
```

Public API:

```cpp
ReadOptions read_options;
Instance inst = ReadInstanceFromFile(filepath, read_options);
```

`main.cpp` now follows the old command-line interface and no longer relies on
the matrix-style toy parser.


## v5 exact one-drone slaves

The new decomposition now has two exact one-drone slave implementations:

- `SingleDroneOrderingCPSatSlave`: OR-Tools CP-SAT, selected by `slave_solver=cp`.
- `SingleDroneOrderingMILPSlave`: Gurobi MILP, selected by `slave_solver=milp`.

Default command-line slave is `cp`.

The old `SingleDroneOrderingSlave` is kept as a buildable fallback when the exact solver libraries are not enabled.


## v6: heuristic baseline removed

From v6 onward:

- `MasterProblem` always builds and runs the Gurobi decomposition master.
- CMake fails if `BBCP_WITH_GUROBI=OFF`.
- The default slave is exact one-drone CP-SAT (`slave_solver=cp`).
- The MILP one-drone slave can be selected with `slave_solver=milp`.
- The old `SingleDroneOrderingSlave` is not used by `SlaveSolverFactory`; it is kept only as a deprecated standalone helper.


## v7 update

Branching/preprocessing/cuts added:

1. Explicit \(y_i\) and \(v_k\).
2. \(t_i+y_i=1\), \(\sum_k a_{ik}=y_i\), \(a_{ik}\le v_k\), \(v_k\ge v_{k+1}\).
3. Every drone must be used.
4. Workload ordering \(W_k\ge W_{k+1}\).
5. Hybrid branch priority and variable hints.
6. Static preprocessing and fixing.
7. Broadcast slave subset cuts to all drones.
8. Energy-cover and pair drone-time cuts.
9. No-good cuts from UB.
