#pragma once

#include "bbcp/core/Instance.h"

namespace bbcp
{

    struct MasterPreprocessStats
    {
        double initial_all_truck_ub = 0.0;
        int fixed_not_drone = 0;
        int fixed_drone_by_energy = 0;
        int fixed_drone_by_roundtrip_ub = 0;
        int fixed_truck_arcs_by_ub = 0;
    };

    double GreedyAllTruckUpperBound(const Instance &inst);

} // namespace bbcp
