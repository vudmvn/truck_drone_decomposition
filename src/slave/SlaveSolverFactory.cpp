#include "bbcp/slave/SlaveSolverFactory.h"

#include <memory>
#include <stdexcept>

#include "bbcp/slave/SingleDroneOrderingCPSatSlave.h"
#include "bbcp/slave/SingleDroneOrderingMILPSlave.h"

namespace bbcp {

std::unique_ptr<ISlaveSolver> CreateSlaveSolver(
    const Instance& inst, SlaveSolverType type, MasterAssignmentMode assignment_mode) {
    if (inst.n_drones <= 0) {
        throw std::invalid_argument("Number of drones must be >= 1");
    }

    if (assignment_mode == MasterAssignmentMode::SlaveAssign) {
        if (type == SlaveSolverType::MILP) {
            throw std::invalid_argument(
                "Unsupported combination: slave-assign+milp; "
                "existing MILP solves one drone only. Use --slave cp.");
        }
#ifdef BBCP_WITH_ORTOOLS
        return std::make_unique<SingleDroneOrderingCPSatSlave>(inst, 0);
#else
        throw std::runtime_error(
            "slave-assign requires OR-Tools K-drone CP-SAT. "
            "Build with -DBBCP_WITH_ORTOOLS=ON; no silent fallback.");
#endif
    }

    if (type == SlaveSolverType::MILP) {
        return std::make_unique<SingleDroneOrderingMILPSlave>(inst);
    }
#ifdef BBCP_WITH_ORTOOLS
    return std::make_unique<SingleDroneOrderingCPSatSlave>(inst, 1);
#else
    throw std::runtime_error(
        "explicit+cp requires OR-Tools. Use --slave milp or rebuild with OR-Tools.");
#endif
}

} // namespace bbcp
