#pragma once
#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"
namespace bbcp {
struct SolveStats{int gurobi_status=2;double lower_bound=0;};
struct FullSolution{bool feasible=true;double objective=10;SolveStats stats;};
struct MasterProblem{MasterProblem(const Instance&,const MasterOptions&){}FullSolution Solve(){return {};} };
}
