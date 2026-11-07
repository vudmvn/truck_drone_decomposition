#pragma once
#include <memory>
#include "bbcp/slave/ISlaveSolver.h"
namespace bbcp{std::unique_ptr<ISlaveSolver>CreateSlaveSolver(const Instance&,SlaveSolverType,MasterAssignmentMode);}
