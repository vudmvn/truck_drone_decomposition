#include "bbcp/slave/SingleDroneOrderingMILPSlave.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "bbcp/slave/SingleDroneOrderingSlave.h"
#include "bbcp/utils/MathUtils.h"

#ifdef BBCP_WITH_GUROBI
#include "gurobi_c++.h"
#endif

namespace bbcp {

SingleDroneOrderingMILPSlave::SingleDroneOrderingMILPSlave(const Instance& inst)
    : inst_(inst) {}

const char* SingleDroneOrderingMILPSlave::Name() const {
    return "single-drone-ordering-milp";
}

SlaveResult SingleDroneOrderingMILPSlave::Solve(
    const std::vector<Node>& drone_customers,
    const SlaveOptions& options
) const {
#ifndef BBCP_WITH_GUROBI
    (void)drone_customers;
    (void)options;
    SlaveResult res;
    res.unknown = true;
    return res;
#else
    SlaveResult res;

    const int m = static_cast<int>(drone_customers.size());

    if (m == 0) {
        res.feasible = true;
        res.optimal = true;
        res.objective = 0;
        res.lower_bound = 0;
        res.lower_bound_valid = true;
        res.chains.push_back({});
        res.swap_before.push_back({});
        return res;
    }

    int64_t total_work = 0;
    int64_t total_energy = 0;
    int64_t max_single = 0;

    for (Node i : drone_customers) {
        if (!inst_.IsCustomer(i) || !inst_.IsDroneFeasible(i)) {
            res.proven_infeasible = true;
            return res;
        }

        if (inst_.energy[i] > inst_.battery_capacity) {
            res.proven_infeasible = true;
            return res;
        }

        total_work += inst_.DroneRoundTripTime(i);
        total_energy += inst_.energy[i];
        max_single = std::max(max_single, inst_.DroneRoundTripTime(i));
    }

    const int64_t min_cycles = CeilDivPositive(total_energy, inst_.battery_capacity);
    const int64_t min_swaps = std::max<int64_t>(0, min_cycles - 1);
    const int64_t lb = std::max(max_single, total_work + inst_.swap_time * min_swaps);

    res.lower_bound = lb;
    res.lower_bound_valid = true;

    if (options.makespan_cutoff >= 0 && lb > options.makespan_cutoff) {
        res.proven_infeasible = true;
        return res;
    }

    // Fast exact case:
    // If swap time is zero, the objective is independent of the number and
    // positions of swaps. Since every customer in drone_customers is already
    // individually drone-feasible, we can reset the battery before every
    // customer at no cost. Therefore the exact optimal value is simply
    // sum_i r_i. This avoids time-limited MILP/CP-SAT calls that return a
    // feasible solution without an optimality proof.
    if (inst_.swap_time == 0) {
        const int64_t exact_obj = total_work;

        if (options.makespan_cutoff >= 0 && exact_obj > options.makespan_cutoff) {
            res.proven_infeasible = true;
            res.lower_bound = exact_obj;
            res.lower_bound_valid = true;
            return res;
        }

        std::vector<Node> chain;
        std::vector<int> swaps;

        chain.reserve(drone_customers.size());
        swaps.reserve(drone_customers.size());

        for (size_t p = 0; p < drone_customers.size(); ++p) {
            chain.push_back(drone_customers[p]);
            swaps.push_back(p == 0 ? 0 : 1);
        }

        res.feasible = true;
        res.optimal = true;
        res.objective = exact_obj;
        res.lower_bound = exact_obj;
        res.lower_bound_valid = true;
        res.chains.push_back(chain);
        res.swap_before.push_back(swaps);
        return res;
    }

    try {
        GRBEnv env(true);
        env.set(GRB_IntParam_OutputFlag, options.log_search ? 1 : 0);
        env.start();

        GRBModel model(env);
        model.set(GRB_StringAttr_ModelName, "single_drone_ordering_milp");

        if (options.time_limit_seconds > 0.0) {
            model.set(GRB_DoubleParam_TimeLimit, options.time_limit_seconds);
        }

        if (options.makespan_cutoff >= 0) {
            // Additional solver-level objective cutoff. The model also
            // contains z <= makespan_cutoff, but Cutoff helps Gurobi prune
            // objective values above the improving region earlier.
            model.set(GRB_DoubleParam_Cutoff,
                      static_cast<double>(options.makespan_cutoff) + 1e-6);
        }

        const double horizon =
            (options.makespan_cutoff >= 0)
                ? static_cast<double>(options.makespan_cutoff)
                : static_cast<double>(total_work + inst_.swap_time * std::max(0, m - 1));

        std::vector<GRBVar> h(m);
        std::vector<GRBVar> s(m);
        std::vector<GRBVar> b(m);
        std::vector<GRBVar> rank(m);
        std::vector<std::vector<GRBVar>> u(m, std::vector<GRBVar>(m));

        for (int p = 0; p < m; ++p) {
            const Node i = drone_customers[p];

            h[p] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "h_" + std::to_string(i));
            s[p] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "s_" + std::to_string(i));
            b[p] = model.addVar(0.0, static_cast<double>(inst_.battery_capacity), 0.0,
                                GRB_CONTINUOUS, "b_" + std::to_string(i));
            rank[p] = model.addVar(0.0, static_cast<double>(m - 1), 0.0,
                                   GRB_CONTINUOUS, "rank_" + std::to_string(i));
        }

        for (int p = 0; p < m; ++p) {
            for (int q = 0; q < m; ++q) {
                if (p == q) continue;

                u[p][q] = model.addVar(
                    0.0, 1.0, 0.0, GRB_BINARY,
                    "u_" + std::to_string(drone_customers[p]) +
                    "_" + std::to_string(drone_customers[q])
                );
            }
        }

        GRBVar z = model.addVar(0.0, horizon, 1.0, GRB_CONTINUOUS, "z_single_drone");

        model.update();

        if (options.makespan_cutoff >= 0) {
            model.addConstr(z <= static_cast<double>(options.makespan_cutoff), "cutoff");
        }

        // Exactly one head.
        {
            GRBLinExpr expr = 0.0;
            for (int p = 0; p < m; ++p) expr += h[p];
            model.addConstr(expr == 1.0, "one_head");
        }

        // Every node is head or has one predecessor.
        for (int q = 0; q < m; ++q) {
            GRBLinExpr expr = h[q];

            for (int p = 0; p < m; ++p) {
                if (p == q) continue;
                expr += u[p][q];
            }

            model.addConstr(expr == 1.0, "in_" + std::to_string(q));
        }

        // At most one successor.
        for (int p = 0; p < m; ++p) {
            GRBLinExpr out = 0.0;

            for (int q = 0; q < m; ++q) {
                if (p == q) continue;
                out += u[p][q];
            }

            model.addConstr(out <= 1.0, "out_" + std::to_string(p));
        }

        // m-1 arcs.
        {
            GRBLinExpr arcs = 0.0;
            for (int p = 0; p < m; ++p) {
                for (int q = 0; q < m; ++q) {
                    if (p == q) continue;
                    arcs += u[p][q];
                }
            }
            model.addConstr(arcs == static_cast<double>(m - 1), "num_arcs");
        }

        const double M_rank = static_cast<double>(m);
        const double M_batt = static_cast<double>(inst_.battery_capacity) + 1.0;

        for (int p = 0; p < m; ++p) {
            const Node i = drone_customers[p];

            // If h[p]=1: s[p]=0, b[p]=E, rank[p]=0.
            model.addConstr(s[p] <= 1.0 - h[p], "head_no_swap_" + std::to_string(i));
            model.addConstr(b[p] >= static_cast<double>(inst_.battery_capacity) -
                                      M_batt * (1.0 - h[p]),
                            "head_batt_lb_" + std::to_string(i));
            model.addConstr(b[p] <= static_cast<double>(inst_.battery_capacity) +
                                      M_batt * (1.0 - h[p]),
                            "head_batt_ub_" + std::to_string(i));
            model.addConstr(rank[p] <= M_rank * (1.0 - h[p]),
                            "head_rank_" + std::to_string(i));

            model.addConstr(b[p] >= static_cast<double>(inst_.energy[i]),
                            "enough_batt_" + std::to_string(i));
        }

        for (int p = 0; p < m; ++p) {
            const Node i = drone_customers[p];

            for (int q = 0; q < m; ++q) {
                if (p == q) continue;

                const Node j = drone_customers[q];

                // If q follows p, q is not head.
                model.addConstr(h[q] <= 1.0 - u[p][q],
                                "not_head_" + std::to_string(i) + "_" + std::to_string(j));

                // Rank.
                model.addConstr(
                    rank[q] >= rank[p] + 1.0 - M_rank * (1.0 - u[p][q]),
                    "rank_" + std::to_string(i) + "_" + std::to_string(j)
                );

                // Symmetry: if decreasing transition, force swap before j.
                if (i > j) {
                    model.addConstr(s[q] >= u[p][q],
                                    "sym_swap_" + std::to_string(i) + "_" + std::to_string(j));
                }

                // no-swap propagation:
                // u[p][q]=1 and s[q]=0 -> b[q] = b[p] - e_i.
                // Implemented with big-M using u and s.
                GRBLinExpr rhs_no_swap_m =
                    M_batt * (2.0 - u[p][q] + s[q]);

                model.addConstr(
                    b[q] >= b[p] - static_cast<double>(inst_.energy[i]) - rhs_no_swap_m,
                    "b_noswap_lb_" + std::to_string(i) + "_" + std::to_string(j)
                );

                model.addConstr(
                    b[q] <= b[p] - static_cast<double>(inst_.energy[i]) + rhs_no_swap_m,
                    "b_noswap_ub_" + std::to_string(i) + "_" + std::to_string(j)
                );

                // swap propagation:
                // u[p][q]=1 and s[q]=1 -> b[q] = E.
                GRBLinExpr rhs_swap_m =
                    M_batt * (2.0 - u[p][q] + (1.0 - s[q]));

                model.addConstr(
                    b[q] >= static_cast<double>(inst_.battery_capacity) - rhs_swap_m,
                    "b_swap_lb_" + std::to_string(i) + "_" + std::to_string(j)
                );

                model.addConstr(
                    b[q] <= static_cast<double>(inst_.battery_capacity) + rhs_swap_m,
                    "b_swap_ub_" + std::to_string(i) + "_" + std::to_string(j)
                );
            }
        }

        GRBLinExpr total_swaps = 0.0;
        for (int p = 0; p < m; ++p) {
            total_swaps += s[p];
        }

        model.addConstr(total_swaps >= static_cast<double>(min_swaps), "min_swaps");
        model.addConstr(total_swaps <= static_cast<double>(m - 1), "max_swaps");

        model.addConstr(
            z == static_cast<double>(total_work) + static_cast<double>(inst_.swap_time) * total_swaps,
            "objective_time"
        );

        GRBLinExpr obj = 0.0;
        obj += z;
        model.setObjective(obj, GRB_MINIMIZE);
        model.optimize();

        const int status = model.get(GRB_IntAttr_Status);
        res.wall_time = model.get(GRB_DoubleAttr_Runtime);

        if (status == GRB_INFEASIBLE || status == GRB_CUTOFF) {
            res.proven_infeasible = true;
            return res;
        }

        if (status != GRB_OPTIMAL && status != GRB_TIME_LIMIT && status != GRB_SUBOPTIMAL) {
            res.unknown = true;
            return res;
        }

        if (model.get(GRB_IntAttr_SolCount) <= 0) {
            res.unknown = true;
            return res;
        }

        res.feasible = true;
        res.optimal = (status == GRB_OPTIMAL);
        res.objective = static_cast<int64_t>(std::llround(model.get(GRB_DoubleAttr_ObjVal)));

        if (res.optimal) {
            res.lower_bound = res.objective;
            res.lower_bound_valid = true;
        } else {
            res.lower_bound = static_cast<int64_t>(std::ceil(model.get(GRB_DoubleAttr_ObjBound)));
            res.lower_bound_valid = true;
        }

        std::vector<Node> chain;
        std::vector<int> swaps;

        int head_pos = -1;
        for (int p = 0; p < m; ++p) {
            if (h[p].get(GRB_DoubleAttr_X) > 0.5) {
                head_pos = p;
                break;
            }
        }

        if (head_pos >= 0) {
            int cur = head_pos;
            std::vector<int> seen(m, 0);

            while (cur >= 0 && !seen[cur]) {
                seen[cur] = 1;
                chain.push_back(drone_customers[cur]);
                swaps.push_back(s[cur].get(GRB_DoubleAttr_X) > 0.5 ? 1 : 0);

                int nxt = -1;
                for (int q = 0; q < m; ++q) {
                    if (q == cur) continue;
                    if (u[cur][q].get(GRB_DoubleAttr_X) > 0.5) {
                        nxt = q;
                        break;
                    }
                }

                cur = nxt;
            }
        }

        res.chains.push_back(chain);
        res.swap_before.push_back(swaps);

        return res;
    } catch (const GRBException& e) {
        std::cerr << "[MILP-SLAVE-GUROBI-ERROR] " << e.getMessage() << "\n";
        res.unknown = true;
        return res;
    }
#endif
}

}  // namespace bbcp
