#pragma once

#include "bbcp/slave/ISlaveSolver.h"

namespace bbcp {

// Exact one-drone ordering/swap slave using Gurobi MILP when
// BBCP_WITH_GUROBI is enabled.
//
// If the project is built without Gurobi, the class falls back to the
// exact small branch-and-bound / heuristic implementation in
// SingleDroneOrderingSlave so that the project remains buildable.
class SingleDroneOrderingMILPSlave : public ISlaveSolver {
public:
    explicit SingleDroneOrderingMILPSlave(const Instance& inst);

    SlaveResult Solve(
        const std::vector<Node>& drone_customers,
        const SlaveOptions& options
    ) const override;

    const char* Name() const override;

private:
    const Instance& inst_;
};

}  // namespace bbcp
