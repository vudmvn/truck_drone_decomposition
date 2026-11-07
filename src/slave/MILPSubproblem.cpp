#include "bbcp/slave/MILPSubproblem.h"

#include "bbcp/slave/CPSubproblem.h"

namespace bbcp {

MILPSubproblem::MILPSubproblem(const Instance& inst) : inst_(inst) {}

SlaveResult MILPSubproblem::Solve(
    const std::vector<Node>& drone_customers,
    const SlaveOptions& options
) const {
    // Solver-independent fallback. Replace with Gurobi MILP implementation.
    CPSubproblem fallback(inst_);
    SlaveResult res = fallback.Solve(drone_customers, options);
    res.optimal = false;
    return res;
}

const char* MILPSubproblem::Name() const {
    return "milp-greedy-fallback";
}

}  // namespace bbcp
