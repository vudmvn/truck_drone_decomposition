#pragma once

#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"
#include "bbcp/core/Solution.h"
#include "bbcp/slave/ISlaveSolver.h"

namespace bbcp {

class LocalSearch {
public:
    LocalSearch(
        const Instance& inst,
        const LocalSearchOptions& options,
        const ISlaveSolver& single_drone_slave
    );

    bool Improve(const FullSolution& current, FullSolution& improved) const;

private:
    const Instance& inst_;
    LocalSearchOptions options_;
    const ISlaveSolver& single_drone_slave_;

    bool EvaluatePerDrone(FullSolution& sol) const;
};

}  // namespace bbcp
