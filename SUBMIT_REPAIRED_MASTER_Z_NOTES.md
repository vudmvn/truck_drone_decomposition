# v11 submitted master z update

This version changes the callback acceptance logic.

If all slave calls return feasible schedules under the cutoff

```text
z_slave <= current_valid_UB - 1
```

then the callback computes:

```cpp
accepted_z = max(truck_value, max_k slave_value_k);
submitted_z = max(master_z, accepted_z);
```

If:

```cpp
accepted_z < best_valid_ub_
```

then the valid original-problem UB is updated with `accepted_z`, but the solution
submitted to Gurobi uses:

```cpp
data_.z = submitted_z;
```

instead of `data_.z = accepted_z`.

This ensures that the submitted master incumbent does not have a master objective
below either:

1. the original candidate master objective, or
2. the slave-checked true accepted objective.

## Feasible but not optimal slave

A feasible slave schedule is now enough to build a valid incumbent. Therefore the
callback no longer aborts when:

```text
slave feasible, but not optimal
slave objective > master z_k
```

In this case:

- the slave feasible objective is used only for incumbent submission / UB update;
- a cut is added only if the slave provides a valid lower bound;
- the slave objective is not used as a lower-bound cut unless optimality is proven.
