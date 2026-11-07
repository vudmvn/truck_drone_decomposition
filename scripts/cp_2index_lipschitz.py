#!/usr/bin/env python3
"""2-Index Anonymous CP-SAT Monolithic Model with Lipschitz Cuts & Valid Inequalities.

Solves the PDSTSP with Battery Swapping as a unified Constraint Programming (CP) model.
Implements the 2-Index Anonymous Formulation from main_with_cp.tex, enhanced with:
1. Per-Customer Bottleneck Lower Bounds
2. Integer Energy-Cover Battery Swaps Bound (Lipschitz step-function)
3. Combined Lipschitz Workload-Makespan Lower Bound
4. Static Pair Battery Incompatibility Cuts
5. Customer Order Symmetry Breaking within Charge Cycles
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

# Try to import ortools, or fall back to known venvs
try:
    from ortools.sat.python import cp_model
except ImportError:
    # Check common virtualenv paths
    possible_venvs = [
        Path("/home/orlab/Minh_ALL/Drone_Battery_Swap/or_env/lib/python3.11/site-packages"),
        Path("/home/orlab/Minh_ALL/Drone_Battery_Swap/or_env/lib/python3.10/site-packages"),
        Path("/home/orlab/Minh_ALL/workbench/.venv/lib/python3.11/site-packages"),
    ]
    found = False
    for p in possible_venvs:
        if p.is_dir() and str(p) not in sys.path:
            sys.path.insert(0, str(p))
            try:
                from ortools.sat.python import cp_model
                found = True
                break
            except ImportError:
                pass
    if not found:
        print("[ERROR] Google OR-Tools is required to run this script.", file=sys.stderr)
        print("Please activate a venv with ortools or run with /home/orlab/Minh_ALL/Drone_Battery_Swap/or_env/bin/python", file=sys.stderr)
        sys.exit(1)


class InstanceData:
    def __init__(self, filepath: str, override_drones: int = -1, override_swap_min: float = -1.0):
        self.filepath = filepath
        self.n_customers = 0
        self.n_drones = 1
        self.n_trucks = 1
        self.truck_cap = 1300.0
        self.drone_cap = 2.27
        self.truck_speed = 30.0
        self.drone_speed = 40.0
        self.drone_endurance_sec = 2160
        self.drone_time_limit_sec = 5400
        self.battery_capacity = 1000
        self.swap_time_sec = 0

        self.coords: List[Tuple[float, float]] = []
        self.demand: List[float] = []

        self._read_instance(filepath)

        if override_drones > 0:
            self.n_drones = override_drones
        if override_swap_min >= 0.0:
            self.swap_time_sec = int(round(override_swap_min * 60.0))

        self._compute_matrices()

    def _read_instance(self, filepath: str):
        with open(filepath, "r", encoding="utf-8") as f:
            lines = [l.strip() for l in f if l.strip()]

        data_rows = []
        for line in lines:
            if "," in line and not line[0].isdigit():
                parts = line.split(",")
                key = parts[0].strip().upper()
                val = parts[1].strip()
                if key == "NUM DRONES": self.n_drones = int(val)
                elif key == "NUM TRUCKS": self.n_trucks = int(val)
                elif key == "TRUCK CAP": self.truck_cap = float(val)
                elif key == "DRONE CAP": self.drone_cap = float(val)
                elif key == "TRUCK SPEED": self.truck_speed = float(val)
                elif key == "DRONE SPEED": self.drone_speed = float(val)
                elif key == "DRONE ENDURANCE": self.drone_endurance_sec = int(round(float(val) * 3600))
                elif key == "DRONE TIME LIMIT": self.drone_time_limit_sec = int(round(float(val) * 3600))
                elif key in ("BATTERY CAPACITY", "BATTERY CAP"): self.battery_capacity = int(val)
                elif key == "SWAP TIME": self.swap_time_sec = int(round(float(val) * 60))
            else:
                parts = line.split()
                if len(parts) >= 4 and parts[0].isdigit():
                    idx = int(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    dem = float(parts[3])
                    data_rows.append((idx, x, y, dem))

        data_rows.sort(key=lambda r: r[0])
        self.n_customers = len(data_rows) - 1
        self.coords = [(r[1], r[2]) for r in data_rows]
        self.demand = [r[3] for r in data_rows]

    def _compute_matrices(self):
        N = self.n_customers + 1
        self.truck_time = [[0] * N for _ in range(N)]
        self.drone_roundtrip = [0] * N
        self.drone_energy = [0] * N
        self.drone_feasible = [False] * N

        for i in range(N):
            for j in range(N):
                if i != j:
                    d = math.hypot(self.coords[i][0] - self.coords[j][0], self.coords[i][1] - self.coords[j][1])
                    t_sec = int(round((d / self.truck_speed) * 3600.0))
                    self.truck_time[i][j] = t_sec

        alpha = 0.217
        beta = 0.185
        xi = 650.0

        for i in range(1, N):
            w = self.demand[i]
            if w > self.drone_cap:
                continue

            d0i = math.hypot(self.coords[0][0] - self.coords[i][0], self.coords[0][1] - self.coords[i][1])
            one_way_sec = int(round((d0i / self.drone_speed) * 3600.0))
            rt_sec = 2 * one_way_sec
            self.drone_roundtrip[i] = rt_sec

            if self.drone_endurance_sec > 0 and rt_sec > self.drone_endurance_sec:
                continue
            if self.drone_time_limit_sec > 0 and rt_sec > self.drone_time_limit_sec:
                continue

            t = float(one_way_sec)
            hrt = 1.0 - (alpha / xi) * 2.0 * t
            if hrt <= 0.0:
                continue

            e = ((alpha * (t * w)) + (2.0 * beta * t)) / hrt / 3600.0
            scaled_e = int(round(e * 1000.0 + 0.5))
            self.drone_energy[i] = scaled_e

            if 0 < scaled_e <= self.battery_capacity:
                self.drone_feasible[i] = True

        self.Cd = [i for i in range(1, N) if self.drone_feasible[i]]


def build_and_solve_cp_2index(
    inst: InstanceData,
    time_limit_sec: float = 3600.0,
    num_workers: int = 8,
    enable_lipschitz: bool = True,
    enable_symmetry_breaking: bool = True,
    enable_pair_cuts: bool = True,
    enable_energy_cover: bool = True,
    log_search: bool = False,
):
    model = cp_model.CpModel()

    N = inst.n_customers + 1
    Cd = inst.Cd
    K = inst.n_drones
    E = inst.battery_capacity
    tau_swap = inst.swap_time_sec

    # Estimate safe makespan upper bound (all-truck tour heuristic upper bound)
    horizon = sum(inst.truck_time[0][i] + inst.truck_time[i][0] for i in range(1, N)) + 3600 * 24
    makespan = model.NewIntVar(0, horizon, "makespan")

    # 1. Truck Routing Variables & Circuit
    x = {}
    for i in range(N):
        for j in range(N):
            x[i, j] = model.NewBoolVar(f"x[{i},{j}]")

    model.AddCircuit([(i, j, x[i, j]) for i in range(N) for j in range(N)])
    model.Add(x[0, 0] == 0)

    truck_travel_time = cp_model.LinearExpr.Sum([
        inst.truck_time[i][j] * x[i, j]
        for i in range(N) for j in range(N) if i != j
    ])
    model.Add(makespan >= truck_travel_time)

    # 2. Drone Anonymous Routing Variables
    y = {i: model.NewBoolVar(f"y[{i}]") for i in Cd}
    u = {(i, j): model.NewBoolVar(f"u[{i},{j}]") for i in Cd for j in Cd if i != j}

    start = {i: model.NewIntVar(0, horizon, f"start[{i}]") for i in Cd}
    end = {i: model.NewIntVar(0, horizon, f"end[{i}]") for i in Cd}
    size = {i: model.NewIntVar(0, horizon, f"size[{i}]") for i in Cd}

    swap = {i: model.NewBoolVar(f"swap[{i}]") for i in Cd}
    battery = {i: model.NewIntVar(0, E, f"battery[{i}]") for i in Cd}
    has_pred = {i: model.NewBoolVar(f"has_pred[{i}]") for i in Cd}

    # Partitioning: Each customer served exactly once (by truck or drone)
    for i in range(1, N):
        truck_in = sum(x[h, i] for h in range(N) if h != i)
        drone_in = y[i] if i in Cd else 0
        model.Add(truck_in + drone_in == 1)

    # Fleet size limit: at most K anonymous chains start without a predecessor
    starts_expr = []
    for i in Cd:
        preds = [u[j, i] for j in Cd if j != i]
        model.Add(sum(preds) == has_pred[i])
        starts_expr.append(y[i] - sum(preds))
    model.Add(sum(starts_expr) <= K)

    # Drone Degree & Temporal Logic
    for i in Cd:
        # Swap requires drone assignment
        model.Add(swap[i] <= y[i])

        # In/out degree at most 1
        model.Add(sum(u[i, j] for j in Cd if j != i) <= y[i])
        model.Add(sum(u[j, i] for j in Cd if j != i) <= y[i])

        # If not served by drone, state variables are 0
        model.Add(size[i] == 0).OnlyEnforceIf(y[i].Not())
        model.Add(end[i] == 0).OnlyEnforceIf(y[i].Not())
        model.Add(start[i] == 0).OnlyEnforceIf(y[i].Not())
        model.Add(battery[i] == 0).OnlyEnforceIf(y[i].Not())

        # If first in chain, start at 0 and no initial swap needed
        model.Add(start[i] == 0).OnlyEnforceIf([y[i], has_pred[i].Not()])
        model.Add(swap[i] == 0).OnlyEnforceIf([y[i], has_pred[i].Not()])
        model.Add(battery[i] == E).OnlyEnforceIf([y[i], has_pred[i].Not()])

        # Precedence chaining
        for j in Cd:
            if i != j:
                model.Add(start[j] == end[i]).OnlyEnforceIf([y[i], y[j], u[i, j]])

        # Sortie duration & makespan
        r_i = inst.drone_roundtrip[i]
        model.Add(size[i] == r_i + tau_swap * swap[i]).OnlyEnforceIf(y[i])
        model.Add(start[i] + size[i] == end[i]).OnlyEnforceIf(y[i])
        model.Add(makespan >= end[i]).OnlyEnforceIf(y[i])

        # Battery tracking
        e_i = inst.drone_energy[i]
        model.Add(battery[i] >= e_i).OnlyEnforceIf(y[i])
        model.Add(battery[i] == E).OnlyEnforceIf([y[i], has_pred[i], swap[i]])

        for j in Cd:
            if i != j:
                e_j = inst.drone_energy[j]
                model.Add(battery[i] == battery[j] - e_j).OnlyEnforceIf([y[i], y[j], u[j, i], swap[i].Not()])

    # =========================================================================
    # ADVANCED VALID INEQUALITIES & LIPSCHITZ BOUNDS
    # =========================================================================

    # 1. Per-Customer Bottleneck Lower Bound (Makespan >= longest drone flight)
    if enable_lipschitz:
        for i in Cd:
            model.Add(makespan >= inst.drone_roundtrip[i] * y[i])

    # 2. Integer Energy-Cover Battery Swaps Bound
    # ceil(sum e_i * y_i / E) - K <= sum swap_i
    if enable_energy_cover and Cd:
        max_cycles = len(Cd)
        q_cycles = model.NewIntVar(0, max_cycles, "q_cycles")
        total_energy_expr = cp_model.LinearExpr.Sum([inst.drone_energy[i] * y[i] for i in Cd])
        model.Add(q_cycles * E >= total_energy_expr)
        total_swaps = cp_model.LinearExpr.Sum([swap[i] for i in Cd])
        model.Add(total_swaps >= q_cycles - K)

    # 3. Combined Lipschitz Workload-Makespan Lower Bound
    # K * makespan >= sum (r_i * y_i) + tau_swap * sum swap_i
    if enable_lipschitz and Cd:
        total_drone_workload = cp_model.LinearExpr.Sum([
            inst.drone_roundtrip[i] * y[i] + tau_swap * swap[i]
            for i in Cd
        ])
        model.Add(K * makespan >= total_drone_workload)

    # 4. Static Pair Battery Incompatibility Cuts
    # If e_i + e_j > E, drone cannot serve j immediately after i without a swap
    if enable_pair_cuts:
        for i in Cd:
            for j in Cd:
                if i != j and inst.drone_energy[i] + inst.drone_energy[j] > E:
                    model.Add(u[i, j] <= swap[j])

    # 5. Customer Order Symmetry Breaking within same charge cycle
    # Unless a swap occurs, order must be strictly ascending (i < j)
    if enable_symmetry_breaking:
        for i in Cd:
            for j in Cd:
                if i != j and i > j:
                    model.Add(u[i, j] <= swap[j])

    # Objective: Minimize overall makespan
    model.Minimize(makespan)

    # Solve
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit_sec
    solver.parameters.num_search_workers = num_workers
    solver.parameters.log_search_progress = log_search

    t0 = time.time()
    status = solver.Solve(model)
    elapsed = time.time() - t0

    result = {
        "status": solver.StatusName(status),
        "runtime_sec": elapsed,
        "feasible": status in (cp_model.OPTIMAL, cp_model.FEASIBLE),
        "optimal": status == cp_model.OPTIMAL,
        "best_ub": solver.ObjectiveValue() if status in (cp_model.OPTIMAL, cp_model.FEASIBLE) else -1,
        "best_lb": solver.BestObjectiveBound() if status in (cp_model.OPTIMAL, cp_model.FEASIBLE) else -1,
    }

    if result["feasible"]:
        if result["best_lb"] > 0:
            result["gap_pct"] = max(0.0, ((result["best_ub"] - result["best_lb"]) / result["best_lb"]) * 100.0)
        else:
            result["gap_pct"] = 0.0

        # Extract truck route
        truck_succ = {}
        for i in range(N):
            for j in range(N):
                if i != j and solver.Value(x[i, j]) == 1:
                    truck_succ[i] = j
        route = [0]
        curr = truck_succ.get(0, 0)
        while curr != 0 and curr is not None and len(route) < N + 2:
            route.append(curr)
            curr = truck_succ.get(curr, 0)
        route.append(0)
        result["truck_route"] = route

        # Extract drone missions
        drone_customers = [i for i in Cd if solver.Value(y[i]) == 1]
        result["drone_customers"] = drone_customers
        result["total_swaps"] = sum(solver.Value(swap[i]) for i in Cd)

    return result


def main():
    parser = argparse.ArgumentParser(description="2-Index Anonymous CP-SAT Solver with Lipschitz Cuts")
    parser.add_argument("instance", help="Path to instance file (.txt)")
    parser.add_argument("--time-limit", type=float, default=3600.0, help="Time limit in seconds (default: 3600)")
    parser.add_argument("--workers", type=int, default=8, help="Number of CP-SAT search workers (default: 8)")
    parser.add_argument("--drones", type=int, default=-1, help="Override number of drones K")
    parser.add_argument("--swap-minutes", type=float, default=-1.0, help="Override swap duration in minutes")
    parser.add_argument("--no-lipschitz", action="store_true", help="Disable Lipschitz workload cuts")
    parser.add_argument("--no-symmetry", action="store_true", help="Disable customer order symmetry breaking")
    parser.add_argument("--no-pair-cuts", action="store_true", help="Disable pair battery incompatibility cuts")
    parser.add_argument("--no-energy-cover", action="store_true", help="Disable energy-cover integer cut")
    parser.add_argument("--log-search", action="store_true", help="Enable CP-SAT search log")

    args = parser.parse_args()

    print("=" * 80)
    print("2-INDEX ANONYMOUS CP-SAT SOLVER WITH LIPSCHITZ INEQUALITIES")
    print(f"Instance         : {args.instance}")
    print(f"Time limit       : {args.time_limit}s")
    print(f"Workers          : {args.workers}")
    print(f"Lipschitz cuts   : {not args.no_lipschitz}")
    print(f"Symmetry breaking: {not args.no_symmetry}")
    print(f"Pair energy cuts : {not args.no_pair_cuts}")
    print(f"Energy cover     : {not args.no_energy_cover}")
    print("=" * 80)

    inst = InstanceData(args.instance, override_drones=args.drones, override_swap_min=args.swap_minutes)
    print(f"[DATA] Nodes={inst.n_customers + 1}, Drone feasible={len(inst.Cd)}, Drones={inst.n_drones}, Swap={inst.swap_time_sec}s")

    res = build_and_solve_cp_2index(
        inst,
        time_limit_sec=args.time_limit,
        num_workers=args.workers,
        enable_lipschitz=not args.no_lipschitz,
        enable_symmetry_breaking=not args.no_symmetry,
        enable_pair_cuts=not args.no_pair_cuts,
        enable_energy_cover=not args.no_energy_cover,
        log_search=args.log_search,
    )

    print("\n" + "=" * 80)
    print(f"STATUS           : {res['status']}")
    print(f"Runtime          : {res['runtime_sec']:.2f}s")
    if res["feasible"]:
        print(f"Best Makespan UB : {res['best_ub']}")
        print(f"Certified LB     : {res['best_lb']}")
        print(f"Gap              : {res.get('gap_pct', 0.0):.2f}%")
        print(f"Truck Route      : {' -> '.join(map(str, res['truck_route']))}")
        print(f"Drone Customers  : {res['drone_customers']} (Count: {len(res['drone_customers'])})")
        print(f"Total Swaps      : {res['total_swaps']}")
    print("=" * 80)


if __name__ == "__main__":
    main()
