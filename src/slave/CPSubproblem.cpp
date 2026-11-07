#include "bbcp/slave/CPSubproblem.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "bbcp/slave/SingleDroneOrderingSlave.h"

namespace bbcp {

CPSubproblem::CPSubproblem(const Instance& inst) : inst_(inst) {}

SlaveResult CPSubproblem::Solve(
    const std::vector<Node>& drone_customers,
    const SlaveOptions& options
) const {
    // Solver-independent fallback: greedy list scheduling across drones.
    // Replace this body with the OR-Tools CP-SAT model when integrating CP.
    SlaveResult res;

    if (drone_customers.empty()) {
        res.feasible = true;
        res.optimal = true;
        res.objective = 0;
        res.lower_bound = 0;
        res.lower_bound_valid = true;
        return res;
    }

    std::vector<std::vector<Node>> sets(inst_.n_drones);
    std::vector<Node> sorted = drone_customers;

    std::sort(sorted.begin(), sorted.end(), [&](Node a, Node b) {
        return inst_.DroneRoundTripTime(a) > inst_.DroneRoundTripTime(b);
    });

    std::vector<int64_t> work(inst_.n_drones, 0);

    for (Node i : sorted) {
        int best = 0;
        for (int k = 1; k < inst_.n_drones; ++k) {
            if (work[k] < work[best]) best = k;
        }
        sets[best].push_back(i);
        work[best] += inst_.DroneRoundTripTime(i);
    }

    SingleDroneOrderingSlave one(inst_);
    int64_t makespan = 0;
    bool all_optimal = true;

    for (int k = 0; k < inst_.n_drones; ++k) {
        SlaveResult r = one.Solve(sets[k], options);
        if (!r.feasible) {
            return r;
        }

        makespan = std::max(makespan, r.objective);
        all_optimal = all_optimal && r.optimal;

        if (!r.chains.empty()) {
            res.chains.push_back(r.chains.front());
            res.swap_before.push_back(r.swap_before.front());
        } else {
            res.chains.push_back({});
            res.swap_before.push_back({});
        }
    }

    res.feasible = true;
    res.optimal = false;  // Greedy distribution across drones is not exact.
    res.objective = makespan;
    res.lower_bound = 0;
    res.lower_bound_valid = false;
    res.unknown = false;

    (void)all_optimal;
    return res;
}

const char* CPSubproblem::Name() const {
    return "cp-greedy-fallback";
}

}  // namespace bbcp
