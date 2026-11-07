#include "bbcp/heuristics/DroneGreedyInsert.h"

namespace bbcp {

bool DroneGreedyInsert::IsDroneCustomerCandidate(const Instance& inst, Node customer) {
    return inst.IsCustomer(customer) && inst.IsDroneFeasible(customer);
}

}  // namespace bbcp
