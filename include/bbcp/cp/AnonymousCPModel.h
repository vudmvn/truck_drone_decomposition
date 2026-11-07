#pragma once

#include <vector>
#include <string>

#include "bbcp/core/Instance.h"
#include "bbcp/core/Solution.h"

namespace bbcp {

struct AnonymousCPOptions {
    double time_limit_seconds = 3600.0;
    int num_search_workers = 8;
    bool enable_lipschitz = true;
    bool enable_symmetry_breaking = true;
    bool enable_pair_cuts = true;
    bool enable_energy_cover = true;
    bool log_search = false;
};

class AnonymousCPModel {
public:
    AnonymousCPModel(const Instance& inst, const AnonymousCPOptions& options);

    FullSolution Solve();

private:
    const Instance& inst_;
    AnonymousCPOptions options_;
};

} // namespace bbcp
