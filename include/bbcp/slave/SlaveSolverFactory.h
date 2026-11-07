#pragma once

#include <memory>

#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"
#include "bbcp/slave/ISlaveSolver.h"

namespace bbcp {

std::unique_ptr<ISlaveSolver>
CreateSlaveSolver(const Instance& inst, SlaveSolverType type, MasterAssignmentMode assignment_mode);

}  // namespace bbcp
