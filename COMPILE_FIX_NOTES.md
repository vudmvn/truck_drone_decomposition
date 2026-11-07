# Compile fix for Gurobi 11 C++ API

## 1. SingleDroneOrderingMILPSlave.cpp

Gurobi expressions such as:

```cpp
M_batt * (2.0 - u[p][q] + s[q])
```

are `GRBLinExpr`, not `double`. Therefore:

```cpp
const double rhs_no_swap_m = ...
const double rhs_swap_m = ...
```

was replaced by:

```cpp
GRBLinExpr rhs_no_swap_m = ...
GRBLinExpr rhs_swap_m = ...
```

## 2. setObjective ambiguity

In Gurobi 11, calling:

```cpp
model.setObjective(z, GRB_MINIMIZE);
model.setObjective(data.z, GRB_MINIMIZE);
```

is ambiguous because `GRBVar` can convert to both `GRBLinExpr` and `GRBQuadExpr`.

Fixed by wrapping objective variables in `GRBLinExpr`:

```cpp
GRBLinExpr obj = 0.0;
obj += z;
model.setObjective(obj, GRB_MINIMIZE);
```

and similarly for `data.z`.
