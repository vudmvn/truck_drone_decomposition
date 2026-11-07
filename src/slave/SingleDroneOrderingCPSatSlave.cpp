#include "bbcp/slave/SingleDroneOrderingCPSatSlave.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "bbcp/utils/MathUtils.h"

#ifdef BBCP_WITH_ORTOOLS
#include "ortools/sat/cp_model.h"
#endif

namespace bbcp {

SingleDroneOrderingCPSatSlave::SingleDroneOrderingCPSatSlave(const Instance& inst, int requested_drones)
    : inst_(inst), requested_drones_(requested_drones) {}

const char* SingleDroneOrderingCPSatSlave::Name() const {
    return requested_drones_ == 1 ? "one-drone-ordering-cpsat" : "k-drone-ordering-cpsat";
}

namespace {

int64_t SafeCeilDiv(int64_t a, int64_t b) {
    if (a <= 0) return 0;
    return (a + b - 1) / b;
}

}  // namespace

SlaveResult SingleDroneOrderingCPSatSlave::Solve(
    const std::vector<Node>& drone_customers,
    const SlaveOptions& options
) const {
    SlaveResult res;

    const int m = static_cast<int>(drone_customers.size());
    const int K = requested_drones_ > 0 ? requested_drones_ : inst_.n_drones;
    const int H = std::min(K, m); // number of nonempty drone chains

    // Empty work set is the ONLY case with zero drone makespan.
    // For 0 < m < K, use H=m nonempty chains and K-H idle drones.
    if (m == 0) {
        res.feasible = true;
        res.optimal = true;
        res.objective = 0;
        res.lower_bound = 0;
        res.lower_bound_valid = true;
        res.chains.assign(std::max(0, K), std::vector<Node>{});
        res.swap_before.assign(std::max(0, K), std::vector<int>{});
        return res;
    }

    if (K <= 0 || inst_.battery_capacity <= 0) {
        res.proven_infeasible = true;
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

    if (K == 1 && inst_.swap_time == 0) {
        if (options.makespan_cutoff >= 0 && total_work > options.makespan_cutoff) {
            res.proven_infeasible = true;
            res.lower_bound = total_work;
            res.lower_bound_valid = true;
            return res;
        }
        res.feasible = true;
        res.optimal = true;
        res.objective = total_work;
        res.lower_bound = total_work;
        res.lower_bound_valid = true;
        res.chains.assign(1, drone_customers);
        res.swap_before.assign(1, std::vector<int>(m, 1));
        if (!res.swap_before.front().empty()) res.swap_before.front().front() = 0;
        return res;
    }

    const int64_t min_cycles = SafeCeilDiv(total_energy, inst_.battery_capacity);
    const int64_t min_swaps = std::max<int64_t>(0, min_cycles - K);
    const int64_t workload_lb = SafeCeilDiv(
        total_work + inst_.swap_time * min_swaps,
        std::max(1, K)
    );
    const int64_t lb = std::max(max_single, workload_lb);

    res.lower_bound = lb;
    res.lower_bound_valid = true;

    if (options.makespan_cutoff >= 0 && lb > options.makespan_cutoff) {
        res.proven_infeasible = true;
        return res;
    }

#ifndef BBCP_WITH_ORTOOLS
    // Without OR-Tools, this exact K-route slave cannot be solved here.
    // Return UNKNOWN but keep the valid lower bound above for safe cutting.
    res.unknown = true;
    return res;
#else
    using namespace operations_research;
    using namespace operations_research::sat;

    const int64_t max_swaps = std::max(0, m - H);
    const int64_t natural_horizon = total_work + inst_.swap_time * max_swaps;
    const int64_t horizon =
        (options.makespan_cutoff >= 0)
            ? options.makespan_cutoff
            : natural_horizon;

    if (horizon < lb) {
        res.proven_infeasible = true;
        return res;
    }

    CpModelBuilder cp;

    std::vector<BoolVar> head(m);
    std::vector<BoolVar> tail(m);
    std::vector<BoolVar> swap_before(m);
    std::vector<IntVar> battery(m);
    std::vector<IntVar> rank(m);
    std::vector<IntVar> completion(m);

    for (int p = 0; p < m; ++p) {
        const Node i = drone_customers[p];

        head[p] = cp.NewBoolVar().WithName("h_" + std::to_string(i));
        tail[p] = cp.NewBoolVar().WithName("tail_" + std::to_string(i));
        swap_before[p] = cp.NewBoolVar().WithName("s_" + std::to_string(i));
        battery[p] = cp.NewIntVar(Domain(0, inst_.battery_capacity))
                         .WithName("b_" + std::to_string(i));
        rank[p] = cp.NewIntVar(Domain(0, m - 1))
                      .WithName("rank_" + std::to_string(i));
        completion[p] = cp.NewIntVar(Domain(0, horizon))
                            .WithName("c_" + std::to_string(i));
    }

    std::vector<std::vector<BoolVar>> u(m, std::vector<BoolVar>(m));
    for (int p = 0; p < m; ++p) {
        for (int q = 0; q < m; ++q) {
            if (p == q) continue;
            u[p][q] = cp.NewBoolVar().WithName(
                "u_" + std::to_string(drone_customers[p]) +
                "_" + std::to_string(drone_customers[q])
            );
        }
    }

    IntVar z = cp.NewIntVar(Domain(0, horizon)).WithName("z_k_drone");

    if (options.makespan_cutoff >= 0) {
        cp.AddLessOrEqual(z, options.makespan_cutoff);
    }

    // Exactly H=min(K,m) non-empty routes; K-H drones may remain idle.
    {
        LinearExpr sum_heads;
        LinearExpr sum_tails;
        for (int p = 0; p < m; ++p) {
            sum_heads += head[p];
            sum_tails += tail[p];
        }
        cp.AddEquality(sum_heads, H);
        cp.AddEquality(sum_tails, H);
    }

    // The smallest-index drone customer is the head of one chain.
    int min_pos = 0;
    for (int p = 1; p < m; ++p) {
        if (drone_customers[p] < drone_customers[min_pos]) {
            min_pos = p;
        }
    }
    cp.AddEquality(head[min_pos], 1);
    cp.AddEquality(swap_before[min_pos], 0);

    // Each customer has exactly one predecessor: dummy source 0 or another customer.
    for (int q = 0; q < m; ++q) {
        LinearExpr pred = head[q];
        for (int p = 0; p < m; ++p) {
            if (p == q) continue;
            pred += u[p][q];
        }
        cp.AddEquality(pred, 1);
    }

    // Each customer has exactly one successor: dummy sink T or another customer.
    for (int p = 0; p < m; ++p) {
        LinearExpr succ = tail[p];
        for (int q = 0; q < m; ++q) {
            if (p == q) continue;
            succ += u[p][q];
        }
        cp.AddEquality(succ, 1);
    }

    // A path cover with m nodes and K paths has m-K internal arcs.
    {
        LinearExpr internal_arcs;
        for (int p = 0; p < m; ++p) {
            for (int q = 0; q < m; ++q) {
                if (p == q) continue;
                internal_arcs += u[p][q];
            }
        }
        cp.AddEquality(internal_arcs, m - H);
    }

    for (int p = 0; p < m; ++p) {
        const Node i = drone_customers[p];

        cp.AddEquality(swap_before[p], 0).OnlyEnforceIf(head[p]);
        cp.AddEquality(battery[p], inst_.battery_capacity).OnlyEnforceIf(head[p]);
        cp.AddEquality(rank[p], 0).OnlyEnforceIf(head[p]);
        cp.AddGreaterOrEqual(completion[p], inst_.DroneRoundTripTime(i))
            .OnlyEnforceIf(head[p]);

        cp.AddGreaterOrEqual(battery[p], inst_.energy[i]);
        cp.AddGreaterOrEqual(z, completion[p]);
    }

    for (int p = 0; p < m; ++p) {
        const Node i = drone_customers[p];

        for (int q = 0; q < m; ++q) {
            if (p == q) continue;

            const Node j = drone_customers[q];

            cp.AddEquality(head[q], 0).OnlyEnforceIf(u[p][q]);
            cp.AddEquality(tail[p], 0).OnlyEnforceIf(u[p][q]);

            // Rank removes disconnected cycles.
            cp.AddGreaterOrEqual(rank[q], rank[p] + 1).OnlyEnforceIf(u[p][q]);

            // Symmetry breaking requested by the user:
            //     u_ij = 0 if i > j unless there is a swap before j.
            // Equivalent CP form: u_ij => s_j when i > j.
            if (i > j) {
                cp.AddEquality(swap_before[q], 1).OnlyEnforceIf(u[p][q]);
            }
            // Redundant, but propagates battery incompatibility early.
            if (inst_.energy[i] + inst_.energy[j] > inst_.battery_capacity) {
                cp.AddEquality(swap_before[q], 1).OnlyEnforceIf(u[p][q]);
            }

            BoolVar no_swap = cp.NewBoolVar().WithName(
                "noswap_" + std::to_string(i) + "_" + std::to_string(j)
            );
            BoolVar swap_arc = cp.NewBoolVar().WithName(
                "swaparc_" + std::to_string(i) + "_" + std::to_string(j)
            );

            // no_swap = u[p][q] AND not swap_before[q].
            cp.AddImplication(no_swap, u[p][q]);
            cp.AddImplication(no_swap, swap_before[q].Not());
            cp.AddBoolOr({u[p][q].Not(), swap_before[q], no_swap});

            // swap_arc = u[p][q] AND swap_before[q].
            cp.AddImplication(swap_arc, u[p][q]);
            cp.AddImplication(swap_arc, swap_before[q]);
            cp.AddBoolOr({u[p][q].Not(), swap_before[q].Not(), swap_arc});

            // battery[p] is remaining battery before serving customer i.
            // If q follows p without a swap, battery before q is residual after i.
            cp.AddEquality(battery[q], battery[p] - inst_.energy[i])
                .OnlyEnforceIf(no_swap);

            // If there is a swap before q, q starts with a full battery.
            cp.AddEquality(battery[q], inst_.battery_capacity)
                .OnlyEnforceIf(swap_arc);

            // Timing propagation.
            cp.AddGreaterOrEqual(
                  completion[q],
                  completion[p] + inst_.DroneRoundTripTime(j))
                .OnlyEnforceIf(no_swap);

            cp.AddGreaterOrEqual(
                  completion[q],
                  completion[p] + inst_.swap_time + inst_.DroneRoundTripTime(j))
                .OnlyEnforceIf(swap_arc);
        }
    }

    LinearExpr total_swaps;
    for (int p = 0; p < m; ++p) {
        total_swaps += swap_before[p];
    }
    cp.AddGreaterOrEqual(total_swaps, min_swaps);
    cp.AddLessOrEqual(total_swaps, max_swaps);

    cp.Minimize(z);

    Model model;
    SatParameters parameters;
    parameters.set_num_search_workers(1);
    parameters.set_log_search_progress(options.log_search);

    if (options.time_limit_seconds > 0.0) {
        parameters.set_max_time_in_seconds(options.time_limit_seconds);
    }

    if (options.stop_after_first_solution) {
        parameters.set_stop_after_first_solution(true);
    }

    model.Add(NewSatParameters(parameters));

    const CpSolverResponse response = SolveCpModel(cp.Build(), &model);

    res.wall_time = response.wall_time();

    // Preserve certified solver bounds even on a time-limited UNKNOWN.
    // We retain the analytic LB and only improve it using the CP-SAT dual bound.
    if (response.status() != CpSolverStatus::MODEL_INVALID) {
        const double dual = response.best_objective_bound();
        if (std::isfinite(dual) && dual >= 0.0 &&
            dual <= static_cast<double>(std::numeric_limits<int64_t>::max() / 8)) {
            const int64_t bound = static_cast<int64_t>(std::ceil(dual - 1e-7));
            res.lower_bound = std::max(res.lower_bound, bound);
            res.lower_bound_valid = true;
        }
    }

    if (response.status() == CpSolverStatus::INFEASIBLE) {
        res.proven_infeasible = true;
        // Infeasibility under a deadline only certifies phi(D) >= deadline+1.
        if (options.makespan_cutoff >= 0) {
            res.lower_bound = std::max(res.lower_bound, options.makespan_cutoff + 1);
            res.lower_bound_valid = true;
        }
        return res;
    }

    if (response.status() != CpSolverStatus::OPTIMAL &&
        response.status() != CpSolverStatus::FEASIBLE) {
        res.unknown = true;
        return res;
    }

    res.feasible = true;
    res.optimal = (response.status() == CpSolverStatus::OPTIMAL);
    res.objective = static_cast<int64_t>(std::llround(response.objective_value()));

    if (res.optimal) {
        res.lower_bound = res.objective;
        res.lower_bound_valid = true;
    } else {
        // Dual bound already extracted above; preserve the strongest LB.
        res.lower_bound_valid = true;
    }

    res.chains.assign(K, {});
    res.swap_before.assign(K, {});

    std::vector<int> succ(m, -1);
    std::vector<int> heads;
    heads.reserve(K);

    for (int p = 0; p < m; ++p) {
        if (SolutionIntegerValue(response, head[p]) == 1) {
            heads.push_back(p);
        }

        for (int q = 0; q < m; ++q) {
            if (p == q) continue;
            if (SolutionIntegerValue(response, u[p][q]) == 1) {
                succ[p] = q;
                break;
            }
        }
    }

    std::sort(heads.begin(), heads.end(), [&](int a, int b) {
        return drone_customers[a] < drone_customers[b];
    });

    for (int route_id = 0; route_id < K && route_id < static_cast<int>(heads.size()); ++route_id) {
        int cur = heads[route_id];
        std::vector<int> seen(m, 0);

        while (cur >= 0 && !seen[cur]) {
            seen[cur] = 1;
            res.chains[route_id].push_back(drone_customers[cur]);
            res.swap_before[route_id].push_back(
                static_cast<int>(SolutionIntegerValue(response, swap_before[cur]))
            );
            cur = succ[cur];
        }
    }

    return res;
#endif
}

}  // namespace bbcp
