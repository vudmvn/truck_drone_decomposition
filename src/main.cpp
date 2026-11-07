#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include "bbcp/master/MasterProblem.h"
#include "bbcp/cp/AnonymousCPModel.h"
#include "bbcp/io/DataReader.h"
#include "bbcp/io/ResultWriter.h"
#include "bbcp/core/Options.h"

namespace {

std::string BaseName(const std::string& path) {
    const size_t pos = path.find_last_of("/\\");
    return pos == std::string::npos ? path : path.substr(pos + 1);
}

bool ParseBool(const std::string& raw) {
    if (raw == "on" || raw == "true" || raw == "1" || raw == "yes") return true;
    if (raw == "off" || raw == "false" || raw == "0" || raw == "no") return false;
    throw std::invalid_argument("Expected on/off, got: " + raw);
}

void PrintVariants() {
#ifdef BBCP_WITH_ORTOOLS
    const char* cp_status = "AVAILABLE";
#else
    const char* cp_status = "UNAVAILABLE (rebuild with BBCP_WITH_ORTOOLS=ON)";
#endif
    std::cout << "\nAssignment/slave combinations for this build:\n";
    std::cout << "  explicit     + cp    : " << cp_status << " (one-drone CP for fixed S_k)\n";
    std::cout << "  explicit     + milp  : AVAILABLE (one-drone Gurobi MILP)\n";
    std::cout << "  slave-assign + cp    : " << cp_status << " (K-drone CP-SAT)\n";
    std::cout << "  slave-assign + milp  : REJECTED (one-drone MILP != K-drone MILP)\n";
    std::cout << "\nTruck: --truck mtz|sec; branching: --branching hybrid|default|...\n"
              << "No silent CP->MILP fallback. UB-based no-good/user cuts disabled.\n"
              << "Local search supported only with explicit master.\n"
              << "Exact claim requires original-problem UB matched by valid Master LB.\n\n";
}

void PrintHelp(const char* exe) {
    std::cout <<
        "Usage: " << exe << " INSTANCE [legacy positional args] [--flags]\n"
        "       " << exe << " --list-variants\n\n"
        "Legacy positional args (backward compatible):\n"
        "  INSTANCE [n_drones] [swap_minutes] [master_time] [slave_time]\n"
        "           [mtz|sec] [result_csv] [branching] [cp|milp] [explicit|slave-assign]\n\n"
        "Named CLI options (override positional args):\n"
        "  --assignment explicit|slave-assign      Default: slave-assign\n"
        "  --slave cp|milp                       Default: cp\n"
        "  --truck mtz|sec                       Default: mtz\n"
        "  --branching default|assignment|cp-aware|ambiguity|drone-benefit|hybrid\n"
        "  --mipnode-cuts on|off                Default: off (safe baseline)\n"
        "  --local-search on|off                 Default: off (explicit only)\n"
        "  --drones INTEGER                      Override instance K\n"
        "  --swap-minutes FLOAT                  Override instance swap time\n"
        "  --time-limit SECONDS                  Master limit, default 3600\n"
        "  --slave-time SECONDS                  Per-slave limit, default 5\n"
        "  --result PATH                         CSV output\n"
        "  --db PATH                             SQLite database output, default: results/results.db\n"
        "  --sqlite PATH                         Alias for --db\n"
        "  --verbose on|off                      Gurobi and callback log\n"
        "  --cut-verbose on|off                  Cut diagnostics\n"
        "  --list-variants                        Print supported combinations\n"
        "  --help                                 Show help\n\n"
        "Note: UB-dependent MIPNODE user cuts and UB-based no-goods are\n"
        "disabled in this correctness-first version.\n\n"
        "Examples:\n"
        "  " << exe << " data/instances/toy_6.txt --assignment explicit --slave milp --truck sec\n"
        "  " << exe << " data/instances/toy_6.txt --assignment explicit --slave cp --truck mtz\n"
        "  " << exe << " data/instances/toy_6.txt --assignment slave-assign --slave cp --truck sec\n";
}

} // namespace

int main(int argc, char** argv) {
    using namespace bbcp;
    try {
        if (argc <= 1) { PrintHelp(argv[0]); return 1; }
        for (int i=1; i<argc; ++i) {
            if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
                PrintHelp(argv[0]); return 0;
            }
            if (std::string(argv[i]) == "--list-variants") {
                PrintVariants(); return 0;
            }
        }

        ReadOptions read_options;
        MasterOptions options;
        options.assignment_mode = MasterAssignmentMode::SlaveAssign;
        options.slave_solver = SlaveSolverType::CP;
        options.subtour_mode = MasterSubtourMode::MTZ;
        options.branching_strategy = BranchingStrategy::Hybrid;
        options.local_search_options.enabled = false;
        options.enable_mipnode_cuts = false;

        // Safe defaults: UB-restricted inequalities are NOT globally valid user cuts.
        options.enable_dynamic_ub_workload_usercut = false;
        options.enable_dynamic_ub_energy_workload_usercut = false;
        options.enable_slave_assign_ub_incompatibility_usercuts = false;

        std::vector<std::string> pos;
        std::vector<std::pair<std::string,std::string>> flags;
        for (int i=1; i<argc; ++i) {
            const std::string item(argv[i]);
            if (item.rfind("--", 0) == 0) {
                if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + item);
                flags.emplace_back(item, argv[++i]);
            } else {
                pos.push_back(item);
            }
        }
        if (pos.empty()) throw std::invalid_argument("INSTANCE data file is required");
        if (pos.size() > 10) throw std::invalid_argument("Too many positional arguments");
        const std::string filepath=pos.at(0);
        options.instance_name=BaseName(filepath);

        if (pos.size()>1) read_options.override_n_drones=std::stoi(pos[1]);
        if (pos.size()>2) read_options.override_swap_minutes=std::stod(pos[2]);
        if (pos.size()>3) options.time_limit_seconds=std::stod(pos[3]);
        if (pos.size()>4) options.slave_options.time_limit_seconds=std::stod(pos[4]);
        if (pos.size()>5) options.subtour_mode=ParseMasterSubtourMode(pos[5]);
        if (pos.size()>6) options.result_file=pos[6];
        if (pos.size()>7) options.branching_strategy=ParseMasterBranchingStrategy(pos[7]);
        if (pos.size()>8) options.slave_solver=ParseSlaveSolverType(pos[8]);
        if (pos.size()>9) options.assignment_mode=ParseMasterAssignmentMode(pos[9]);

        std::string solver_model = "decomposition";

        for (const auto& flag:flags) {
            const std::string& name=flag.first;
            const std::string& val=flag.second;
            if (name=="--assignment" || name=="--assignment-mode")
                options.assignment_mode=ParseMasterAssignmentMode(val);
            else if (name=="--model") solver_model = val;
            else if (name=="--slave") options.slave_solver=ParseSlaveSolverType(val);
            else if (name=="--truck") options.subtour_mode=ParseMasterSubtourMode(val);
            else if (name=="--branching") options.branching_strategy=ParseMasterBranchingStrategy(val);
            else if (name=="--mipnode-cuts") options.enable_mipnode_cuts=ParseBool(val);
            else if (name=="--local-search") options.local_search_options.enabled=ParseBool(val);
            else if (name=="--drones") read_options.override_n_drones=std::stoi(val);
            else if (name=="--swap-minutes") read_options.override_swap_minutes=std::stod(val);
            else if (name=="--time-limit") options.time_limit_seconds=std::stod(val);
            else if (name=="--slave-time") options.slave_options.time_limit_seconds=std::stod(val);
            else if (name=="--result") options.result_file=val;
            else if (name=="--db" || name=="--sqlite") options.sqlite_db_path=val;
            else if (name=="--verbose") options.verbose=ParseBool(val);
            else if (name=="--cut-verbose") options.cut_verbose=ParseBool(val);
            else throw std::invalid_argument("Unknown option: " + name);
        }
        if (options.time_limit_seconds <= 0.0 || options.slave_options.time_limit_seconds<=0.0)
            throw std::invalid_argument("Time limits must be strictly positive");
        if (read_options.override_n_drones != -1 && read_options.override_n_drones < 1)
            throw std::invalid_argument("--drones must be >=1");
        if (read_options.override_swap_minutes != -1.0 &&
            read_options.override_swap_minutes < 0.0)
            throw std::invalid_argument("--swap-minutes cannot be negative");
        if (options.assignment_mode==MasterAssignmentMode::SlaveAssign &&
            options.slave_solver==SlaveSolverType::MILP)
            throw std::invalid_argument(
                "slave-assign+milp is unsupported. Use --assignment slave-assign --slave cp, "
                "or --assignment explicit --slave milp.");
        if (options.assignment_mode==MasterAssignmentMode::SlaveAssign &&
            options.local_search_options.enabled)
            throw std::invalid_argument(
                "--local-search on requires --assignment explicit: LocalSearch evaluates one-drone S_k.");

        options.slave_options.phase2_time_limit_seconds=options.slave_options.time_limit_seconds;
        options.local_search_options.time_limit_seconds=options.slave_options.time_limit_seconds;
        options.slave_options.stop_after_first_solution=false;
        options.slave_options.log_search=false;
        options.slave_assign_usercut_verbose=options.cut_verbose;
        options.use_branching_var_hints=true;

        Instance inst=ReadInstanceFromFile(filepath,read_options);
        if(inst.n_drones<=0) throw std::invalid_argument("Instance must have K>=1");
        if (solver_model == "cp-2index" || solver_model == "cp-anonymous" || solver_model == "monolithic-cp" || solver_model == "cp") {
            AnonymousCPOptions cp_opts;
            cp_opts.time_limit_seconds = options.time_limit_seconds;
            cp_opts.num_search_workers = 8;
            cp_opts.enable_lipschitz = true;
            cp_opts.enable_symmetry_breaking = true;
            cp_opts.enable_pair_cuts = true;
            cp_opts.enable_energy_cover = true;
            cp_opts.log_search = options.verbose;

            std::cout << "[CONFIG] model=cp-2index-anonymous K=" << inst.n_drones
                      << " time_limit=" << cp_opts.time_limit_seconds << "s\n";
            std::cout << "[ENHANCEMENTS] lipschitz_bounds=on energy_cover=on symmetry_breaking=on pair_cuts=on\n";

            AnonymousCPModel cp_solver(inst, cp_opts);
            FullSolution sol = cp_solver.Solve();
            if (sol.feasible) {
                sol.stats.best_valid_ub = sol.objective;
                std::cout << "\n=== 2-Index Anonymous CP-SAT finished ===\n";
                std::cout << "Best valid UB: " << sol.objective << "\n";
                ResultWriter::WriteCSV(options.result_file, inst, options, sol);
                ResultWriter::WriteSolutionText("results/solutions/solution.txt", inst, sol);
                if (!options.sqlite_db_path.empty()) {
                    ResultWriter::WriteSQLite(options.sqlite_db_path, inst, options, sol);
                }
                std::cout << "[RESULT] feasible UB=" << sol.objective
                          << " master_status=" << (sol.feasible ? 2 : 11)
                          << " LB=" << sol.stats.lower_bound << "\n";
                return 0;
            }
            std::cout << "[STATUS] CP-SAT stopped without solution.\n";
            return 3;
        }

        std::cout<<"[CONFIG] assignment="<<MasterAssignmentModeName(options.assignment_mode)
                 <<" slave="<<SlaveSolverTypeName(options.slave_solver)
                 <<" truck="<<MasterSubtourModeName(options.subtour_mode)
                 <<" branching="<<BranchingStrategyName(options.branching_strategy)
                 <<" mipnode_cuts="<<options.enable_mipnode_cuts
                 <<" local_search="<<options.local_search_options.enabled
                 <<" K="<<inst.n_drones<<"\n";
        std::cout<<"[SAFETY] UB-based user cuts/no-good cuts disabled; "
                    "UNKNOWN is not an infeasibility certificate.\n";
        MasterProblem solver(inst,options);
        FullSolution sol=solver.Solve();
        if (sol.feasible) {
            std::cout << "[RESULT] feasible UB=" << sol.objective
                      << " master_status=" << sol.stats.gurobi_status
                      << " LB=" << sol.stats.lower_bound << "\n";
            return 0;
        }
        return 3;
    } catch (const std::exception& e) {
        std::cerr<<"[ERROR] "<<e.what()<<"\n";
        return 2;
    }
}
