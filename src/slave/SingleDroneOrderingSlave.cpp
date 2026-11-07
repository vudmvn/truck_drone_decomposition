// Deprecated helper from previous versions.
// The decomposition master no longer uses this class through SlaveSolverFactory.
// Use SingleDroneOrderingCPSatSlave or SingleDroneOrderingMILPSlave instead.
// This file is kept only for tests/prototyping and for small standalone checks.
//
// This file implements the reduced slave described in docs/model_master_slave.tex.
//
// Input:
//   S_k = fixed set of customers assigned by the master to one drone k.
//
// Decision:
//   order the customers in S_k and decide where battery swaps occur.
//
// Because every drone task is modeled as depot -> customer -> depot, the
// total flight time sum_i r_i is fixed for S_k. The only optimization part is
// minimizing the number of battery segments/swaps needed to pack customer
// energy requirements into batteries of capacity E.
//
// Objective:
//   D_k(S_k) = sum_{i in S_k} r_i + tau * number_of_swaps.
//
// The current implementation solves the energy packing part exactly for small
// sets and uses first-fit decreasing as a fallback for larger sets.
// Replace this by a CP-SAT ordering model if the future drone task time depends
// on sequence-dependent travel times.

#include "bbcp/slave/SingleDroneOrderingSlave.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <vector>

#include "bbcp/utils/MathUtils.h"

namespace bbcp {

namespace {

struct CycleSolution {
    int bins = 0;
    std::vector<std::vector<Node>> cycles;
};

CycleSolution FirstFitDecreasing(
    const Instance& inst,
    std::vector<Node> customers
) {
    std::sort(customers.begin(), customers.end(), [&](Node a, Node b) {
        return inst.energy[a] > inst.energy[b];
    });

    std::vector<int64_t> load;
    std::vector<std::vector<Node>> cycles;

    for (Node i : customers) {
        bool inserted = false;
        for (size_t b = 0; b < load.size(); ++b) {
            if (load[b] + inst.energy[i] <= inst.battery_capacity) {
                load[b] += inst.energy[i];
                cycles[b].push_back(i);
                inserted = true;
                break;
            }
        }

        if (!inserted) {
            load.push_back(inst.energy[i]);
            cycles.push_back({i});
        }
    }

    return {static_cast<int>(cycles.size()), cycles};
}

CycleSolution ExactBinPackingSmall(
    const Instance& inst,
    std::vector<Node> customers,
    int incumbent_bins
) {
    std::sort(customers.begin(), customers.end(), [&](Node a, Node b) {
        return inst.energy[a] > inst.energy[b];
    });

    const int m = static_cast<int>(customers.size());
    int best_bins = incumbent_bins;
    std::vector<std::vector<Node>> best_cycles;

    std::vector<int64_t> load;
    std::vector<std::vector<Node>> cycles;

    std::function<void(int)> dfs = [&](int pos) {
        if (static_cast<int>(load.size()) >= best_bins) {
            return;
        }

        if (pos == m) {
            best_bins = static_cast<int>(load.size());
            best_cycles = cycles;
            return;
        }

        Node i = customers[pos];

        // Try existing bins. Symmetry: do not try two bins with equal load.
        int64_t last_tried_load = -1;
        for (size_t b = 0; b < load.size(); ++b) {
            if (load[b] == last_tried_load) continue;
            if (load[b] + inst.energy[i] <= inst.battery_capacity) {
                last_tried_load = load[b];
                load[b] += inst.energy[i];
                cycles[b].push_back(i);

                dfs(pos + 1);

                cycles[b].pop_back();
                load[b] -= inst.energy[i];
            }
        }

        // Open new bin.
        load.push_back(inst.energy[i]);
        cycles.push_back({i});
        dfs(pos + 1);
        cycles.pop_back();
        load.pop_back();
    };

    dfs(0);

    if (best_cycles.empty()) {
        return FirstFitDecreasing(inst, customers);
    }

    return {best_bins, best_cycles};
}

}  // namespace

SingleDroneOrderingSlave::SingleDroneOrderingSlave(const Instance& inst)
    : inst_(inst) {}

SlaveResult SingleDroneOrderingSlave::Solve(
    const std::vector<Node>& drone_customers,
    const SlaveOptions& options
) const {
    SlaveResult res;

    if (drone_customers.empty()) {
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

    CycleSolution ffd = FirstFitDecreasing(inst_, drone_customers);
    CycleSolution sol = ffd;
    bool exact = false;

    if (drone_customers.size() <= 26) {
        sol = ExactBinPackingSmall(inst_, drone_customers, ffd.bins);
        exact = true;
    }

    const int swaps = std::max(0, sol.bins - 1);
    const int64_t obj = total_work + inst_.swap_time * swaps;

    if (options.makespan_cutoff >= 0 && obj > options.makespan_cutoff) {
        if (exact) {
            res.proven_infeasible = true;
        } else {
            res.unknown = true;
        }
        return res;
    }

    res.feasible = true;
    res.optimal = exact;
    res.objective = obj;
    if (exact) {
        res.lower_bound = obj;
        res.lower_bound_valid = true;
    }

    // Decode cycles into one chain. A swap is placed before the first customer
    // of every cycle after the first one.
    std::vector<Node> chain;
    std::vector<int> swaps_before;

    for (size_t c = 0; c < sol.cycles.size(); ++c) {
        const auto& cycle = sol.cycles[c];
        for (size_t p = 0; p < cycle.size(); ++p) {
            chain.push_back(cycle[p]);
            swaps_before.push_back((c > 0 && p == 0) ? 1 : 0);
        }
    }

    res.chains.push_back(chain);
    res.swap_before.push_back(swaps_before);

    return res;
}

const char* SingleDroneOrderingSlave::Name() const {
    return "single-drone-ordering";
}

}  // namespace bbcp
