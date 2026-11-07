#include "bbcp/slave/SlaveSolverFactory.h"

#include <memory>
#include <stdexcept>

#include "bbcp/slave/SingleDroneOrderingCPSatSlave.h"
#include "bbcp/slave/SingleDroneOrderingMILPSlave.h"

namespace bbcp {

std::unique_ptr<ISlaveSolver>
CreateSlaveSolver(const Instance& inst, SlaveSolverType type) {
    switch (type) {
    case SlaveSolverType::CP:
#ifdef BBCP_WITH_ORTOOLS
        // Default exact slave for the new decomposition.
        return std::make_unique<SingleDroneOrderingCPSatSlave>(inst);
#else
        // OR-Tools was not compiled in. Since v6+ always requires Gurobi,
        // keep the method exact by using the Gurobi one-drone MILP slave
        // instead of returning UNKNOWN from a missing CP-SAT backend.
        return std::make_unique<SingleDroneOrderingMILPSlave>(inst);
#endif

    case SlaveSolverType::MILP:
        return std::make_unique<SingleDroneOrderingMILPSlave>(inst);

    case SlaveSolverType::SingleDroneOrdering:
#ifdef BBCP_WITH_ORTOOLS
        return std::make_unique<SingleDroneOrderingCPSatSlave>(inst);
#else
        return std::make_unique<SingleDroneOrderingMILPSlave>(inst);
#endif
    }

    throw std::runtime_error("Unknown slave solver type");
}

}  // namespace bbcp
