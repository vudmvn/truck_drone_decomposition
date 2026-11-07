#pragma once

#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"

namespace bbcp
{

    struct MasterModelData;

    /**
     * Applies Gurobi branching priorities and optional variable hints for the
     * master problem.
     *
     * The old BranchingStrategySetter::Apply(inst, options) interface could not
     * change Gurobi branching because it did not receive the GRBVar containers.
     * The overload below receives MasterModelData and therefore can set attributes
     * such as GRB_IntAttr_BranchPriority and GRB_DoubleAttr_VarHintVal directly.
     */
    class BranchingStrategySetter
    {
    public:
        // Backward-compatible no-op interface.
        static void Apply(const Instance &inst, const MasterOptions &options);

        // Main implementation. Call this after all master variables have been
        // created and after model.update().
        static void Apply(
            const Instance &inst,
            const MasterOptions &options,
            MasterModelData &data,
            double initial_ub);
    };

} // namespace bbcp
