# v9 slave proof fix for SWAP TIME = 0

The abort

```text
[ERROR] A slave call did not provide enough proof to verify or cut the master incumbent.
```

happens when a one-drone slave returns a feasible schedule, but does not prove
optimality and its lower bound is not strong enough to cut the master incumbent.

For instances with `SWAP TIME,0`, the exact one-drone slave value is immediate:

```text
D*(S) = sum_{i in S} r_i
```

because battery swaps can be inserted before every customer at zero cost. If each
customer is individually drone-feasible, the schedule is feasible and optimal.

This version adds a fast exact path to both:

```text
src/slave/SingleDroneOrderingMILPSlave.cpp
src/slave/SingleDroneOrderingCPSatSlave.cpp
```

before calling Gurobi/CP-SAT. It also adds more detailed diagnostics in
`MasterCallback.cpp` if an unresolved slave proof still occurs.
