# Truck–Drone Master–Slave Decomposition Solver

This version always runs the **exact Gurobi decomposition master**. The previous
solver-independent heuristic baseline has been removed from `MasterProblem`.

The decomposition is:

```text
Master:
  - decides truck route;
  - decides customer served by truck or drone;
  - decides which drone k serves each drone customer.

Slave:
  - for each drone k, receives S_k = {i : a_ik = 1};
  - orders customers in S_k;
  - chooses battery-swap positions.
```

Default slave:

```text
cp -> exact one-drone OR-Tools CP-SAT slave
```

Alternative:

```text
milp -> exact one-drone Gurobi MILP slave
```

## Build

Use the command below:

```bash
cmake -S . -B build \
  -DBBCP_WITH_GUROBI=ON \
  -DBBCP_WITH_ORTOOLS=ON \
  -DGUROBI_MAIN_LIB=gurobi120

cmake --build build -j
```

If your Gurobi library uses a different name, change `GUROBI_MAIN_LIB`, for example:

```bash
-DGUROBI_MAIN_LIB=gurobi110
-DGUROBI_MAIN_LIB=gurobi1103
-DGUROBI_MAIN_LIB=gurobi120
-DGUROBI_MAIN_LIB=gurobi125
```

If CMake cannot find OR-Tools, set:

```bash
export ORTOOLS_HOME=/path/to/ortools
```

If CMake cannot find Gurobi, set:

```bash
export GUROBI_HOME=/path/to/gurobi/linux64
```

## Run

```bash
./build/truck_drone_solver data/instances/legacy_toy_6.txt 2 3 3600 5 cut results/csv/result.csv hybrid cp
```

MILP slave:

```bash
./build/truck_drone_solver data/instances/legacy_toy_6.txt 2 3 3600 5 cut results/csv/result.csv hybrid milp
```

## Important invariant

The original-problem upper bound is updated only by a complete slave-checked
solution:

\[
z_{\text{accepted}}=\max\{z_T,\max_k D_k(S_k)\}.
\]

The master objective alone is never used as a valid original-problem UB.

## Main files

```text
src/master/MasterProblem.cpp
src/master/MasterModelBuilder.cpp
src/master/MasterCallback.cpp
src/master/MasterCuts.cpp
src/slave/SingleDroneOrderingCPSatSlave.cpp
src/slave/SingleDroneOrderingMILPSlave.cpp
src/slave/SlaveSolverFactory.cpp
src/io/DataReader.cpp
```


## v7 branching, preprocessing, and cuts

Implemented in `MasterModelBuilder`:

- added \(y_i\): customer \(i\) served by some drone;
- added \(v_k\): drone \(k\) is used;
- added constraints:
  - \(t_i+y_i=1\);
  - \(\sum_k a_{ik}=y_i\);
  - \(a_{ik}\le v_k\);
  - \(v_k\ge v_{k+1}\);
  - every drone is used: \(v_k=1\), \(\sum_i a_{ik}\ge 1\);
- kept workload ordering \(W_k\ge W_{k+1}\);
- added hybrid branch priorities;
- added variable hints for \(y_i\), \(a_{ik}\), and \(x_{ij}\);
- preprocessing fixes:
  - non-drone-feasible customers;
  - drone sorties exceeding incumbent UB;
  - truck arcs impossible under incumbent UB;
- static cuts:
  - truck inclusion lower bounds;
  - pair energy-cover cuts;
  - pair drone-time cuts;
  - pair no-good cuts from incumbent UB.

Implemented in `MasterCallback`:

- broadcast slave subset cuts to all drones;
- broadcast no-good cuts from slave infeasibility under UB.


## v10 slave logging and cutoff

The callback now prints detailed information about every slave solution returned
by CP-SAT/MILP:

```text
[SLAVE-CALL]
[SLAVE-RESULT]
[SLAVE-SUMMARY]
```

Every slave call is explicitly bounded by:

\[
z_{\text{slave}}\le \UB-1.
\]

For the MILP slave, this is enforced both as a model constraint and through
Gurobi's objective `Cutoff` parameter.


## v11 repaired incumbent submission

When all slaves are feasible, the callback now computes:

\[
z_{\mathrm{accepted}}=\max\{z_T,\max_k z_k^{\mathrm{slave}}\},
\]

and submits the incumbent to Gurobi with:

\[
z_{\mathrm{submitted}}=\max\{z_{\mathrm{master}},z_{\mathrm{accepted}}\}.
\]

The valid UB is still updated by \(z_{\mathrm{accepted}}\), but the submitted
master variable `z` is set to \(z_{\mathrm{submitted}}\).

Feasible but non-optimal slave schedules no longer trigger abort. They are used
to build a valid incumbent, while cuts are added only from proven lower bounds.


## v12 CP fallback when OR-Tools is not available

If the code is built with:

```bash
-DBBCP_WITH_ORTOOLS=OFF
```

then the requested `cp` slave is automatically redirected to the exact Gurobi
MILP one-drone slave. The solver prints:

```text
[SLAVE] requested=cp, actual=single-drone-ordering-milp
```

This prevents `[SLAVE-RESULT] unknown=1, obj=N/A, lb=N/A, wall_time=0` caused by
a missing OR-Tools backend.
