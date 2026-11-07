#pragma once

#include <vector>

#include "bbcp/core/Instance.h"
#include "bbcp/core/Solution.h"

namespace bbcp {

class DroneGreedyInsert {
public:
    static bool IsDroneCustomerCandidate(const Instance& inst, Node customer);
};

}  // namespace bbcp
