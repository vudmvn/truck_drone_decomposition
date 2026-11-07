# v12 CP fallback and slave logging clarification

The log

```text
[SLAVE-RESULT] feasible=0, optimal=0, proven_infeasible=0, unknown=1, obj=N/A, lb=N/A, wall_time=0
```

means the slave did not actually solve the subproblem and returned UNKNOWN.
The common cause is building with:

```bash
-DBBCP_WITH_ORTOOLS=OFF
```

while running the default/requested `cp` slave.

From v12 onward:

- if OR-Tools is compiled in, `cp` uses `SingleDroneOrderingCPSatSlave`;
- if OR-Tools is not compiled in, `cp` automatically redirects to the exact
  Gurobi `SingleDroneOrderingMILPSlave`;
- `MasterProblem` prints both the requested and actual slave backend:

```text
[SLAVE] requested=cp, actual=single-drone-ordering-milp
```

The CP-SAT class itself also has a direct no-OR-Tools safety path:
for `SWAP TIME = 0`, it returns the exact value

```text
D*(S) = sum_i r_i
```

instead of UNKNOWN.
