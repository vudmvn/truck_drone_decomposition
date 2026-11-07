#pragma once

#include <vector>

#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"
#include "bbcp/slave/SlaveResult.h"

namespace bbcp {

class ISlaveSolver {
public:
    virtual ~ISlaveSolver() = default;

    virtual SlaveResult Solve(
        const std::vector<Node>& drone_customers,
        const SlaveOptions& options
    ) const = 0;

    virtual const char* Name() const = 0;
};

}  // namespace bbcp
