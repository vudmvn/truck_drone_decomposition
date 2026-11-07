#include "bbcp/heuristics/TruckTwoOpt.h"

#include <algorithm>

namespace bbcp {

std::vector<Node> TruckTwoOpt::Improve(
    const Instance& inst,
    const std::vector<Node>& route,
    int max_passes
) {
    if (route.size() <= 4) return route;

    std::vector<Node> best = route;
    bool improved = true;
    int pass = 0;

    while (improved && pass < max_passes) {
        improved = false;
        ++pass;

        for (int i = 1; i + 2 < static_cast<int>(best.size()); ++i) {
            for (int k = i + 1; k + 1 < static_cast<int>(best.size()); ++k) {
                Node a = best[i - 1];
                Node b = best[i];
                Node c = best[k];
                Node d = best[k + 1];

                int64_t old_cost = inst.truck_time[a][b] + inst.truck_time[c][d];
                int64_t new_cost = inst.truck_time[a][c] + inst.truck_time[b][d];

                if (new_cost < old_cost) {
                    std::reverse(best.begin() + i, best.begin() + k + 1);
                    improved = true;
                    break;
                }
            }
            if (improved) break;
        }
    }

    return best;
}

}  // namespace bbcp
