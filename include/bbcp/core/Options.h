#pragma once

#include <cstdint>
#include <string>

namespace bbcp
{

    enum class SlaveSolverType
    {
        CP,
        MILP,
        SingleDroneOrdering
    };

    enum class MasterSubtourMode
    {
        MTZ,
        SEC,
        LazyCut = SEC
    };

    enum class MasterAssignmentMode
    {
        Explicit,
        SlaveAssign,
        CP2Index
    };

    enum class BranchingStrategy
    {
        Default,
        AssignmentFirst,
        CPAware,
        AmbiguityFirst,
        DroneBenefit,
        Hybrid
    };

    struct SlaveOptions
    {
        double time_limit_seconds = 5.0;
        double phase2_time_limit_seconds = 10.0;

        bool log_search = false;
        bool stop_after_first_solution = false;

        int64_t makespan_cutoff = -1;
        bool add_cutoff_valid_inequalities = true;
    };

    struct LocalSearchOptions
    {
        bool enabled = true;
        int max_rounds = 100;
        int max_moves_per_round = 100000;
        double time_limit_seconds = 5.0;
    };

    struct MasterOptions
    {
        std::string instance_name;
        std::string result_file = "results/csv/result.csv";
        std::string sqlite_db_path = "results/results.db";

        double time_limit_seconds = 3600.0;
        double mip_gap = 0.0;
        double big_m = 1e7;

        bool verbose = true;
        bool cut_verbose = false;

        MasterSubtourMode subtour_mode = MasterSubtourMode::MTZ;
        MasterAssignmentMode assignment_mode = MasterAssignmentMode::Explicit;
        BranchingStrategy branching_strategy = BranchingStrategy::Hybrid;
        SlaveSolverType slave_solver = SlaveSolverType::CP;

        bool enable_mipnode_cuts = false;

        bool enable_dynamic_ub_workload_usercut = false; // unsafe without a global cutoff proof
        bool enable_dynamic_ub_energy_workload_usercut = false; // same
        double dynamic_ub_workload_usercut_min_improvement = 1.0;

        // ------------------------------------------------------------
        // Slave-assign MIPNODE aggregate drone user cuts.
        // These use only y_i, Q, and zD and therefore apply only in
        // assignment_mode == MasterAssignmentMode::SlaveAssign.
        // ------------------------------------------------------------
        bool enable_slave_assign_aggregate_drone_usercuts = true;
        bool enable_slave_assign_energy_cover_usercuts = true;
        bool enable_slave_assign_workload_usercuts = true;
        bool enable_slave_assign_ub_incompatibility_usercuts = false; // UB no-good not globally valid
        bool enable_slave_assign_activated_truck_sec_usercuts = false;

        int slave_assign_usercut_max_cuts_per_node = 20;
        int slave_assign_usercut_max_subset_size = 8;
        double slave_assign_usercut_y_threshold = 0.20;
        double slave_assign_usercut_min_violation = 1e-6;
        bool slave_assign_usercut_verbose = false;

        bool enable_callback_schedule_local_search = true;
        bool callback_schedule_ls_verbose = true;
        int callback_schedule_ls_max_rounds = 50;
        int callback_schedule_ls_max_moves_per_round = 100000;
        double callback_schedule_ls_min_improvement = 1.0;

        bool use_branching_var_hints = true;

        SlaveOptions slave_options;
        LocalSearchOptions local_search_options;
    };

    SlaveSolverType ParseSlaveSolverType(const std::string &raw);
    std::string SlaveSolverTypeName(SlaveSolverType type);

    MasterSubtourMode ParseMasterSubtourMode(const std::string &raw);
    std::string MasterSubtourModeName(MasterSubtourMode mode);

    MasterAssignmentMode ParseMasterAssignmentMode(const std::string &raw);
    std::string MasterAssignmentModeName(MasterAssignmentMode mode);

    BranchingStrategy ParseBranchingStrategy(const std::string &raw);
    std::string BranchingStrategyName(BranchingStrategy strategy);

    inline BranchingStrategy ParseMasterBranchingStrategy(const std::string &raw)
    {
        return ParseBranchingStrategy(raw);
    }

    inline std::string MasterBranchingStrategyName(BranchingStrategy strategy)
    {
        return BranchingStrategyName(strategy);
    }

} // namespace bbcp
