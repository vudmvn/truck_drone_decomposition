#pragma once

#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"
#include "bbcp/core/Solution.h"

namespace bbcp
{

    class MasterProblem
    {
    public:
        MasterProblem(const Instance &inst, const MasterOptions &options);

        FullSolution Solve();

    private:
        const Instance &inst_;
        MasterOptions options_;
    };

} // namespace bbcp
