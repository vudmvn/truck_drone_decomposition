#pragma once

#include "bbcp/slave/ISlaveSolver.h"

namespace bbcp {

// Exact one-drone ordering/swap slave using OR-Tools CP-SAT when
// BBCP_WITH_ORTOOLS is enabled.
//
// If the project is built without OR-Tools, the class falls back to the
// exact small branch-and-bound / heuristic implementation in
// SingleDroneOrderingSlave so that the project remains buildable.
class SingleDroneOrderingCPSatSlave : public ISlaveSolver {
public:
    explicit SingleDroneOrderingCPSatSlave(const Instance& inst, int requested_drones = 0);

    SlaveResult Solve(
        const std::vector<Node>& drone_customers,
        const SlaveOptions& options
    ) const override;

    const char* Name() const override;

private:
    const Instance& inst_;
    int requested_drones_ = 0; // 1 for explicit mode; 0 means K for slave-assign
};

}  // namespace bbcp
