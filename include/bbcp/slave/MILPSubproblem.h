#pragma once
#include "bbcp/slave/ISlaveSolver.h"

namespace bbcp {

class MILPSubproblem : public ISlaveSolver {
public:
    explicit MILPSubproblem(const Instance& inst);

    SlaveResult Solve(
        const std::vector<Node>& drone_customers,
        const SlaveOptions& options
    ) const override;

    const char* Name() const override;

private:
    const Instance& inst_;
};

}  // namespace bbcp
