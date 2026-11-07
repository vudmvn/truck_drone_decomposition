#pragma once
#include "bbcp/slave/ISlaveSolver.h"

namespace bbcp {

class CPSubproblem : public ISlaveSolver {
public:
    explicit CPSubproblem(const Instance& inst);

    SlaveResult Solve(
        const std::vector<Node>& drone_customers,
        const SlaveOptions& options
    ) const override;

    const char* Name() const override;

private:
    const Instance& inst_;
};

}  // namespace bbcp
