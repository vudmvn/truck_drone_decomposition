#include "bbcp/heuristics/InitialSolution.h"

#include <limits>
#include <vector>

#include "bbcp/heuristics/TruckTwoOpt.h"
#include "bbcp/utils/RouteUtils.h"

namespace bbcp {

FullSolution InitialSolution::GreedyAllTruck(const Instance& inst) {
    FullSolution sol;
    sol.feasible = true;

    std::vector<bool> used(inst.n + 1, false);
    Node cur = inst.start_depot;

    sol.truck_route.push_back(cur);

    for (int step = 0; step < inst.n; ++step) {
        Node best = -1;
        int64_t best_time = std::numeric_limits<int64_t>::max();

        for (Node i = 1; i <= inst.n; ++i) {
            if (used[i]) continue;
            if (inst.truck_time[cur][i] < best_time) {
                best_time = inst.truck_time[cur][i];
                best = i;
            }
        }

        if (best == -1) break;

        sol.truck_route.push_back(best);
        sol.truck_customers.push_back(best);
        used[best] = true;
        cur = best;
    }

    sol.truck_route.push_back(inst.end_depot);
    sol.truck_route = TruckTwoOpt::Improve(inst, sol.truck_route);

    sol.truck_customers.clear();
    for (Node v : sol.truck_route) {
        if (inst.IsCustomer(v)) sol.truck_customers.push_back(v);
    }

    sol.truck_value = ComputeRouteTime(inst, sol.truck_route);
    sol.drone_value = 0.0;
    sol.objective = sol.truck_value;
    sol.per_drone_customers.assign(inst.n_drones, {});
    sol.drone_schedule.chains.assign(inst.n_drones, {});
    sol.drone_schedule.swap_before.assign(inst.n_drones, {});
    sol.drone_schedule.makespan = 0.0;

    return sol;
}

}  // namespace bbcp
