#include "bbcp/master/MasterPreprocess.h"

#include "bbcp/heuristics/InitialSolution.h"

namespace bbcp {

double GreedyAllTruckUpperBound(const Instance& inst) {
    return InitialSolution::GreedyAllTruck(inst).objective;
}

}  // namespace bbcp
