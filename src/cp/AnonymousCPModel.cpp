#include "bbcp/cp/AnonymousCPModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#ifdef BBCP_WITH_ORTOOLS
#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model_solver.h"
#include "ortools/sat/model.h"
#include "ortools/sat/sat_parameters.pb.h"
#endif

namespace bbcp {

AnonymousCPModel::AnonymousCPModel(const Instance& inst, const AnonymousCPOptions& options)
    : inst_(inst), options_(options) {}

FullSolution AnonymousCPModel::Solve() {
    FullSolution sol;
    sol.feasible = false;

#ifndef BBCP_WITH_ORTOOLS
    std::cerr << "[ERROR] AnonymousCPModel requires BBCP_WITH_ORTOOLS.\n";
    return sol;
#else
    using namespace operations_research;
    using namespace operations_research::sat;

    const int K = inst_.n_drones;
    const int64_t E = inst_.battery_capacity;
    const int64_t tau_swap = inst_.swap_time;

    std::vector<int> Cd;
    for (int i = 1; i <= inst_.n; ++i) {
        if (inst_.drone_feasible[i]) {
            Cd.push_back(i);
        }
    }
    const int m = static_cast<int>(Cd.size());

    int64_t horizon = 0;
    for (int i = 1; i <= inst_.n; ++i) {
        horizon += inst_.truck_time[0][i] + inst_.truck_time[i][0];
    }
    horizon += 3600 * 24;

    CpModelBuilder cp;
    IntVar makespan = cp.NewIntVar(Domain(0, horizon)).WithName("makespan");

    // 1. Truck Routing Variables & Circuit Constraint
    // We map nodes 0..inst_.n
    const int n_nodes = inst_.n + 1;
    std::vector<std::vector<BoolVar>> x(n_nodes, std::vector<BoolVar>(n_nodes));
    CircuitConstraint circuit = cp.AddCircuitConstraint();

    for (int i = 0; i < n_nodes; ++i) {
        for (int j = 0; j < n_nodes; ++j) {
            x[i][j] = cp.NewBoolVar().WithName("x_" + std::to_string(i) + "_" + std::to_string(j));
            if (i == 0 && j == 0) {
                // Depot self loop is forbidden
                cp.AddEquality(x[i][j], 0);
            }
            circuit.AddArc(i, j, x[i][j]);
        }
    }

    LinearExpr truck_travel_time;
    for (int i = 0; i < n_nodes; ++i) {
        for (int j = 0; j < n_nodes; ++j) {
            if (i != j) {
                truck_travel_time += inst_.truck_time[i][j] * x[i][j];
            }
        }
    }
    cp.AddGreaterOrEqual(makespan, truck_travel_time);

    // 2. Drone Anonymous Routing Variables
    std::vector<BoolVar> y(n_nodes);
    std::vector<std::vector<BoolVar>> u(n_nodes, std::vector<BoolVar>(n_nodes));
    std::vector<IntVar> start(n_nodes);
    std::vector<IntVar> end(n_nodes);
    std::vector<IntVar> size(n_nodes);
    std::vector<BoolVar> swap(n_nodes);
    std::vector<IntVar> battery(n_nodes);
    std::vector<BoolVar> has_pred(n_nodes);

    for (int i : Cd) {
        y[i] = cp.NewBoolVar().WithName("y_" + std::to_string(i));
        swap[i] = cp.NewBoolVar().WithName("swap_" + std::to_string(i));
        has_pred[i] = cp.NewBoolVar().WithName("has_pred_" + std::to_string(i));
        start[i] = cp.NewIntVar(Domain(0, horizon)).WithName("start_" + std::to_string(i));
        end[i] = cp.NewIntVar(Domain(0, horizon)).WithName("end_" + std::to_string(i));
        size[i] = cp.NewIntVar(Domain(0, horizon)).WithName("size_" + std::to_string(i));
        battery[i] = cp.NewIntVar(Domain(0, E)).WithName("battery_" + std::to_string(i));

        for (int j : Cd) {
            if (i != j) {
                u[i][j] = cp.NewBoolVar().WithName("u_" + std::to_string(i) + "_" + std::to_string(j));
            }
        }
    }

    // Partitioning: Each customer served exactly once (by truck or drone)
    for (int i = 1; i <= inst_.n; ++i) {
        LinearExpr truck_in;
        for (int h = 0; h < n_nodes; ++h) {
            if (h != i) {
                truck_in += x[h][i];
            }
        }
        if (inst_.drone_feasible[i]) {
            cp.AddEquality(truck_in + y[i], 1);
        } else {
            cp.AddEquality(truck_in, 1);
        }
    }

    // Fleet size limit: at most K anonymous chains start without a predecessor
    LinearExpr starts_sum;
    for (int i : Cd) {
        LinearExpr preds;
        for (int j : Cd) {
            if (j != i) {
                preds += u[j][i];
            }
        }
        cp.AddEquality(preds, has_pred[i]);
        starts_sum += y[i] - preds;
    }
    cp.AddLessOrEqual(starts_sum, K);

    // Drone Degree & Temporal Logic
    for (int i : Cd) {
        // Swap requires drone assignment
        cp.AddLessOrEqual(swap[i], y[i]);

        // In/out degree at most 1
        LinearExpr sum_out;
        LinearExpr sum_in;
        for (int j : Cd) {
            if (j != i) {
                sum_out += u[i][j];
                sum_in += u[j][i];
            }
        }
        cp.AddLessOrEqual(sum_out, y[i]);
        cp.AddLessOrEqual(sum_in, y[i]);

        // If not served by drone, state variables are 0
        cp.AddEquality(size[i], 0).OnlyEnforceIf(y[i].Not());
        cp.AddEquality(end[i], 0).OnlyEnforceIf(y[i].Not());
        cp.AddEquality(start[i], 0).OnlyEnforceIf(y[i].Not());
        cp.AddEquality(battery[i], 0).OnlyEnforceIf(y[i].Not());

        // First in chain
        cp.AddEquality(start[i], 0).OnlyEnforceIf({y[i], has_pred[i].Not()});
        cp.AddEquality(swap[i], 0).OnlyEnforceIf({y[i], has_pred[i].Not()});
        cp.AddEquality(battery[i], E).OnlyEnforceIf({y[i], has_pred[i].Not()});

        // Precedence chaining
        for (int j : Cd) {
            if (i != j) {
                cp.AddEquality(start[j], end[i]).OnlyEnforceIf({y[i], y[j], u[i][j]});
            }
        }

        // Sortie duration & makespan
        const int64_t r_i = inst_.drone_roundtrip_time[i];
        cp.AddEquality(size[i], r_i + tau_swap * swap[i]).OnlyEnforceIf(y[i]);
        cp.AddEquality(start[i] + size[i], end[i]).OnlyEnforceIf(y[i]);
        cp.AddGreaterOrEqual(makespan, end[i]).OnlyEnforceIf(y[i]);

        // Battery tracking
        const int64_t e_i = inst_.energy[i];
        cp.AddGreaterOrEqual(battery[i], e_i).OnlyEnforceIf(y[i]);
        cp.AddEquality(battery[i], E).OnlyEnforceIf({y[i], has_pred[i], swap[i]});

        for (int j : Cd) {
            if (i != j) {
                const int64_t e_j = inst_.energy[j];
                cp.AddEquality(battery[i], battery[j] - e_j).OnlyEnforceIf({y[i], y[j], u[j][i], swap[i].Not()});
            }
        }
    }

    // =========================================================================
    // ADVANCED VALID INEQUALITIES & LIPSCHITZ BOUNDS
    // =========================================================================

    // 1. Per-Customer Bottleneck Lower Bound
    if (options_.enable_lipschitz) {
        for (int i : Cd) {
            cp.AddGreaterOrEqual(makespan, inst_.drone_roundtrip_time[i] * y[i]);
        }
    }

    // 2. Integer Energy-Cover Battery Swaps Bound
    if (options_.enable_energy_cover && !Cd.empty()) {
        IntVar q_cycles = cp.NewIntVar(Domain(0, m)).WithName("q_cycles");
        LinearExpr total_energy;
        for (int i : Cd) {
            total_energy += inst_.energy[i] * y[i];
        }
        cp.AddGreaterOrEqual(q_cycles * E, total_energy);

        LinearExpr total_swaps;
        for (int i : Cd) {
            total_swaps += swap[i];
        }
        cp.AddGreaterOrEqual(total_swaps, q_cycles - K);
    }

    // 3. Combined Lipschitz Workload-Makespan Lower Bound
    if (options_.enable_lipschitz && !Cd.empty()) {
        LinearExpr total_drone_workload;
        for (int i : Cd) {
            total_drone_workload += inst_.drone_roundtrip_time[i] * y[i] + tau_swap * swap[i];
        }
        cp.AddGreaterOrEqual(K * makespan, total_drone_workload);
    }

    // 4. Static Pair Battery Incompatibility Cuts
    if (options_.enable_pair_cuts) {
        for (int i : Cd) {
            for (int j : Cd) {
                if (i != j && inst_.energy[i] + inst_.energy[j] > E) {
                    cp.AddLessOrEqual(u[i][j], swap[j]);
                }
            }
        }
    }

    // 5. Customer Order Symmetry Breaking within charge cycle
    if (options_.enable_symmetry_breaking) {
        for (int i : Cd) {
            for (int j : Cd) {
                if (i != j && i > j) {
                    cp.AddLessOrEqual(u[i][j], swap[j]);
                }
            }
        }
    }

    // Objective: Minimize overall makespan
    cp.Minimize(makespan);

    // Solve configuration
    Model model;
    SatParameters params;
    params.set_max_time_in_seconds(options_.time_limit_seconds);
    params.set_num_search_workers(options_.num_search_workers);
    params.set_log_search_progress(options_.log_search);
    model.Add(NewSatParameters(params));

    const auto t_start = std::chrono::steady_clock::now();
    const CpSolverResponse response = SolveCpModel(cp.Build(), &model);
    const auto t_end = std::chrono::steady_clock::now();
    sol.stats.runtime_sec = std::chrono::duration<double>(t_end - t_start).count();

    if (response.status() == CpSolverStatus::OPTIMAL ||
        response.status() == CpSolverStatus::FEASIBLE) {
        sol.feasible = true;
        sol.objective = response.objective_value();
        sol.stats.best_valid_ub = sol.objective;
        sol.stats.lower_bound = response.best_objective_bound();

        if (sol.stats.lower_bound > 0.0) {
            sol.stats.mip_gap = std::max(0.0, (sol.objective - sol.stats.lower_bound) / sol.stats.lower_bound);
            sol.stats.final_valid_gap_percent = sol.stats.mip_gap * 100.0;
        }

        // Extract Truck Route
        std::vector<int> truck_next(n_nodes, 0);
        for (int i = 0; i < n_nodes; ++i) {
            for (int j = 0; j < n_nodes; ++j) {
                if (i != j && SolutionBooleanValue(response, x[i][j])) {
                    truck_next[i] = j;
                }
            }
        }
        std::vector<Node> route = {0};
        int curr = truck_next[0];
        while (curr != 0 && static_cast<int>(route.size()) < n_nodes + 2) {
            route.push_back(curr);
            curr = truck_next[curr];
        }
        route.push_back(0);
        sol.truck_route = route;

        // Truck customers
        for (int i = 1; i <= inst_.n; ++i) {
            if (truck_next[i] != i && SolutionBooleanValue(response, x[i][i]) == false) {
                sol.truck_customers.push_back(i);
            }
        }

        // Drone customers
        for (int i : Cd) {
            if (SolutionBooleanValue(response, y[i])) {
                sol.drone_customers.push_back(i);
            }
        }
    }

    return sol;
#endif
}

} // namespace bbcp
