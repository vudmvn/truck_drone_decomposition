#pragma once
#include <stdexcept>
#include <string>
namespace bbcp {
enum class SlaveSolverType{CP,MILP,SingleDroneOrdering};
enum class MasterSubtourMode{MTZ,SEC};
enum class MasterAssignmentMode{Explicit,SlaveAssign};
enum class BranchingStrategy{Default,Hybrid};
struct SlaveOptions{double time_limit_seconds=5,phase2_time_limit_seconds=5;bool stop_after_first_solution=false,log_search=false;};
struct LocalSearchOptions{bool enabled=false;double time_limit_seconds=5;};
struct MasterOptions {
 std::string instance_name, result_file="results/csv/result.csv";double time_limit_seconds=3600;
 MasterAssignmentMode assignment_mode=MasterAssignmentMode::Explicit;
 SlaveSolverType slave_solver=SlaveSolverType::CP;
 MasterSubtourMode subtour_mode=MasterSubtourMode::MTZ;
 BranchingStrategy branching_strategy=BranchingStrategy::Hybrid;
 LocalSearchOptions local_search_options;SlaveOptions slave_options;
 bool enable_mipnode_cuts=false,enable_dynamic_ub_workload_usercut=false, enable_dynamic_ub_energy_workload_usercut=false,enable_slave_assign_ub_incompatibility_usercuts=false;
 bool verbose=true,cut_verbose=false,slave_assign_usercut_verbose=false,use_branching_var_hints=true;
};
inline MasterAssignmentMode ParseMasterAssignmentMode(std::string x){if(x=="explicit")return MasterAssignmentMode::Explicit;if(x=="slave-assign")return MasterAssignmentMode::SlaveAssign;throw std::runtime_error("bad assignment");}
inline SlaveSolverType ParseSlaveSolverType(std::string x){if(x=="cp")return SlaveSolverType::CP;if(x=="milp")return SlaveSolverType::MILP;throw std::runtime_error("bad solver");}
inline MasterSubtourMode ParseMasterSubtourMode(std::string x){if(x=="sec")return MasterSubtourMode::SEC;if(x=="mtz")return MasterSubtourMode::MTZ;throw std::runtime_error("bad subtour");}
inline BranchingStrategy ParseMasterBranchingStrategy(std::string x){if(x=="hybrid")return BranchingStrategy::Hybrid;if(x=="default")return BranchingStrategy::Default;throw std::runtime_error("bad branching");}
inline std::string MasterAssignmentModeName(MasterAssignmentMode x){return x==MasterAssignmentMode::Explicit?"explicit":"slave-assign";}
inline std::string SlaveSolverTypeName(SlaveSolverType x){return x==SlaveSolverType::CP?"cp":"milp";}
inline std::string MasterSubtourModeName(MasterSubtourMode x){return x==MasterSubtourMode::MTZ?"mtz":"sec";}
inline std::string BranchingStrategyName(BranchingStrategy x){return x==BranchingStrategy::Hybrid?"hybrid":"default";}
}
