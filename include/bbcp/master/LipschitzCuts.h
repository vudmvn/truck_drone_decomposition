#pragma once

#include <algorithm>

#include "bbcp/core/Instance.h"

namespace bbcp {

// Valid for independent 0->i->0 drone tasks with individually feasible
// energies e_i <= B, identical drones, and a nonnegative constant swap time.
// Inserting customer i into ANY complete solution for a subset requires at most
// r_i + swap_time additional time on one drone. Thus, for certified L(S),
//   z_D + sum_{i in S} min(r_i+swap_time, L(S)) * (1-y_i) >= L(S)
// and the equivalent per-drone inequality are globally valid.
struct LipschitzCuts {
    static double MakespanDrop(const Instance& inst, Node customer,
                               double certified_lower_bound) {
        if (certified_lower_bound <= 0.0) return 0.0;
        const double addition_cost =
            static_cast<double>(inst.DroneRoundTripTime(customer)) +
            static_cast<double>(inst.swap_time);
        return std::min(certified_lower_bound, std::max(0.0, addition_cost));
    }
};

} // namespace bbcp
