#include "bbcp/heuristics/LocalSearch.h"

#include <algorithm>
#include <limits>

#include "bbcp/heuristics/TruckTwoOpt.h"
#include "bbcp/utils/RouteUtils.h"

namespace bbcp {

LocalSearch::LocalSearch(
    const Instance& inst,
    const LocalSearchOptions& options,
    const ISlaveSolver& single_drone_slave
) : inst_(inst), options_(options), single_drone_slave_(single_drone_slave) {}

bool LocalSearch::EvaluatePerDrone(FullSolution& sol) const {
    sol.drone_schedule.chains.assign(inst_.n_drones, {});
    sol.drone_schedule.swap_before.assign(inst_.n_drones, {});
    sol.drone_schedule.makespan = 0.0;
    sol.drone_schedule.swap_count = 0;

    SlaveOptions slave_opts;
    slave_opts.time_limit_seconds = options_.time_limit_seconds;

    for (int k = 0; k < inst_.n_drones; ++k) {
        SlaveResult r = single_drone_slave_.Solve(sol.per_drone_customers[k], slave_opts);
        if (!r.feasible) return false;

        sol.drone_schedule.makespan =
            std::max(sol.drone_schedule.makespan, static_cast<double>(r.objective));

        if (!r.chains.empty()) {
            sol.drone_schedule.chains[k] = r.chains.front();
            sol.drone_schedule.swap_before[k] = r.swap_before.front();

            for (int v : sol.drone_schedule.swap_before[k]) {
                if (v != 0) sol.drone_schedule.swap_count++;
            }
        }
    }

    sol.drone_value = sol.drone_schedule.makespan;
    sol.objective = std::max(sol.truck_value, sol.drone_value);
    return true;
}

bool LocalSearch::Improve(const FullSolution& current, FullSolution& improved) const {
    if (!options_.enabled) {
        improved = current;
        return false;
    }

    FullSolution best = current;
    bool any_improved = false;

    for (int round = 0; round < options_.max_rounds; ++round) {
        bool round_improved = false;
        FullSolution best_candidate = best;

        for (Node customer : best.truck_customers) {
            if (!inst_.IsDroneFeasible(customer)) continue;

            for (int k = 0; k < inst_.n_drones; ++k) {
                FullSolution cand = best;

                cand.truck_route = RemoveCustomerFromRoute(cand.truck_route, customer);
                cand.truck_route = TruckTwoOpt::Improve(inst_, cand.truck_route);
                cand.truck_value = ComputeRouteTime(inst_, cand.truck_route);

                cand.truck_customers.clear();
                for (Node v : cand.truck_route) {
                    if (inst_.IsCustomer(v)) cand.truck_customers.push_back(v);
                }

                cand.per_drone_customers[k].push_back(customer);
                cand.drone_customers.push_back(customer);

                if (!EvaluatePerDrone(cand)) continue;

                if (cand.objective + 1e-9 < best_candidate.objective) {
                    best_candidate = cand;
                    round_improved = true;
                }
            }
        }

        if (!round_improved) break;

        best = best_candidate;
        any_improved = true;
    }

    improved = best;
    return any_improved;
}

}  // namespace bbcp
