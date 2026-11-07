#include <cstdlib>
#include <iostream>
#include <string>

#include "bbcp/master/MasterProblem.h"
#include "bbcp/io/DataReader.h"
#include "bbcp/core/Options.h"

namespace
{

    std::string BaseName(const std::string &path)
    {
        const size_t pos = path.find_last_of("/\\");
        if (pos == std::string::npos)
        {
            return path;
        }
        return path.substr(pos + 1);
    }

} // namespace

int main(int argc, char **argv)
{
    using namespace bbcp;

    if (argc < 2)
    {
        std::cerr
            << "Usage:\n"
            << "  " << argv[0]
            << " <data_file> [n_drones] [swap_minutes] [master_time] [slave_time]"
            << " [master_mode] [result_csv] [branching] [slave_solver] [assignment_mode]\n\n"
            << "master_mode options:\n"
            << "  mtz                    use MTZ constraints in the master (default)\n"
            << "  sec                    use lazy subtour-elimination constraints\n"
            << "  cut/lazy/lazycut       aliases of sec\n\n"
            << "branching options:\n"
            << "  default\n"
            << "  assignment\n"
            << "  cp-aware\n"
            << "  ambiguity\n"
            << "  drone-benefit\n"
            << "  hybrid\n\n"
            << "slave_solver options:\n"
            << "  cp                     use exact OR-Tools CP-SAT slave (default)\n"
            << "  milp                   use exact Gurobi MILP slave\n"
            << "  single-drone-ordering  alias of cp for the new decomposition\n\n"
            << "assignment_mode options:\n"
            << "  explicit               master has a_i_k, q_k, z_k variables (default)\n"
            << "  slave-assign           master has only x/y/Q; slave assigns customers to K drones\n"
            << "Examples:\n"
            << "  " << argv[0]
            << " data.txt 4 3 3600 5 mtz results.csv ambiguity cp\n"
            << "  " << argv[0]
            << " data.txt 4 3 3600 5 sec results.csv ambiguity cp\n"
            << "  " << argv[0]
            << " data.txt 4 3 3600 5 sec results.csv hybrid cp slave-assign\n\n"
            << "This version always runs the exact Gurobi decomposition master.\n"
            << "Build with:\n"
            << "  cmake -S . -B build -DBBCP_WITH_GUROBI=ON -DBBCP_WITH_ORTOOLS=ON\n";

        return 1;
    }

    const std::string filepath = argv[1];

    ReadOptions read_options;

    if (argc >= 3)
    {
        read_options.override_n_drones = std::atoi(argv[2]);
    }

    if (argc >= 4)
    {
        read_options.override_swap_minutes = std::atof(argv[3]);
    }

    MasterOptions master_options;
    master_options.instance_name = BaseName(filepath);

    if (argc >= 5)
    {
        master_options.time_limit_seconds = std::atof(argv[4]);
    }

    if (argc >= 6)
    {
        master_options.slave_options.time_limit_seconds = std::atof(argv[5]);
        master_options.slave_options.phase2_time_limit_seconds = std::atof(argv[5]);
        master_options.local_search_options.time_limit_seconds = std::atof(argv[5]);
    }

    if (argc >= 7)
    {
        master_options.subtour_mode = ParseMasterSubtourMode(argv[6]);
    }

    if (argc >= 8)
    {
        master_options.result_file = argv[7];
    }

    if (argc >= 9)
    {
        master_options.branching_strategy =
            ParseMasterBranchingStrategy(argv[8]);
    }

    if (argc >= 10)
    {
        master_options.slave_solver =
            ParseSlaveSolverType(argv[9]);
    }
    else
    {
        // Default: exact CP-SAT slave.
        master_options.slave_solver = SlaveSolverType::CP;
    }

    if (argc >= 11)
    {
        master_options.assignment_mode =
            ParseMasterAssignmentMode(argv[10]);
    }

    master_options.use_branching_var_hints = true;

    // Hard-coded switch; not exposed as a command-line argument.
    // When enabled, the callback schedules the dynamic UB workload cut
    // whenever the valid incumbent UB decreases by at least one unit.
    master_options.enable_dynamic_ub_workload_usercut = true;
    master_options.enable_dynamic_ub_energy_workload_usercut = true;
    master_options.dynamic_ub_workload_usercut_min_improvement = 1.0;

    master_options.verbose = true;
    master_options.callback_schedule_ls_verbose = false;
    master_options.enable_callback_schedule_local_search = false;
    master_options.cut_verbose = false;
    master_options.slave_options.log_search = false;
    master_options.slave_options.stop_after_first_solution = false;

    master_options.assignment_mode = MasterAssignmentMode::SlaveAssign;
    master_options.enable_mipnode_cuts = true;
    master_options.slave_assign_usercut_verbose = true;
    try
    {
        Instance inst = ReadInstanceFromFile(filepath, read_options);

        MasterProblem master(inst, master_options);
        FullSolution sol = master.Solve();

        return sol.feasible ? 0 : 3;
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 2;
    }
}
