#pragma once

#include <cstdint>
#include <vector>

#include "bbcp/core/Types.h"

namespace bbcp {

struct SlaveResult {
    bool feasible = false;
    bool optimal = false;
    bool proven_infeasible = false;
    bool unknown = false;

    int64_t objective = -1;
    int64_t lower_bound = -1;
    bool lower_bound_valid = false;

    double wall_time = 0.0;

    std::vector<std::vector<Node>> chains;
    std::vector<std::vector<int>> swap_before;
};

}  // namespace bbcp
