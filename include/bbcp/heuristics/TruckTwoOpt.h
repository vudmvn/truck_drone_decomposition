#pragma once

#include <vector>

#include "bbcp/core/Instance.h"

namespace bbcp {

class TruckTwoOpt {
public:
    static std::vector<Node> Improve(
        const Instance& inst,
        const std::vector<Node>& route,
        int max_passes = 100
    );
};

}  // namespace bbcp
