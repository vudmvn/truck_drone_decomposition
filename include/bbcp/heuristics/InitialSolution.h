#pragma once

#include "bbcp/core/Instance.h"
#include "bbcp/core/Solution.h"

namespace bbcp {

class InitialSolution {
public:
    static FullSolution GreedyAllTruck(const Instance& inst);
};

}  // namespace bbcp
