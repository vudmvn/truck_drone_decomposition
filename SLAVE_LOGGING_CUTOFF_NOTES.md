# v10 slave logging and cutoff update

## Added slave-result logging

For every MIPSOL and every drone, the callback now prints:

```text
[SLAVE-CALL]
  drone id
  |S|
  cutoff
  S

[SLAVE-RESULT]
  feasible
  optimal
  proven_infeasible
  unknown
  objective
  lower bound
  wall time
  returned chain
  swap positions
```

It also prints the final checked value:

```text
[SLAVE-SUMMARY]
  truck
  drone_makespan
  accepted_z
  current_UB
```

## Enforced slave upper bound

Every slave call is now explicitly restricted to the improving region:

```text
z_slave <= current_valid_UB - 1
```

The callback computes:

```cpp
slave_cutoff = floor(best_valid_ub_ - 1)
slave_opts.makespan_cutoff = min(existing_cutoff, slave_cutoff)
```

and passes it to the slave.

## MILP slave

The MILP slave already had:

```cpp
z <= makespan_cutoff
```

This version additionally sets the Gurobi parameter:

```cpp
GRB_DoubleParam_Cutoff = makespan_cutoff + 1e-6
```

and treats `GRB_CUTOFF` like proven cutoff infeasibility.

## CP-SAT slave

The CP-SAT slave uses:

```cpp
z in Domain(0, makespan_cutoff)
z <= makespan_cutoff
```
