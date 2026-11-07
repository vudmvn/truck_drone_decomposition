#pragma once

#include <limits>
#include <vector>

#include "bbcp/core/Types.h"

namespace bbcp
{

    struct SolveStats
    {
        int gurobi_status = -1;

        double master_obj = -1.0;
        double lower_bound = -1.0;
        double mip_gap = -1.0;
        double runtime_sec = -1.0;

        double initial_valid_ub = std::numeric_limits<double>::infinity();
        double best_valid_ub = std::numeric_limits<double>::infinity();
        double best_master_lb_seen = -std::numeric_limits<double>::infinity();

        double final_valid_gap = -1.0;
        double final_valid_gap_percent = -1.0;

        double time_to_best_solution_sec = -1.0;
        double time_to_1pct_gap_sec = -1.0;

        double first_1pct_gap_value = -1.0;
        double first_1pct_gap_percent = -1.0;
        double first_1pct_gap_bound = -1.0;

        int valid_incumbent_updates = 0;
    };

    struct DroneSchedule
    {
        std::vector<std::vector<Node>> chains;
        std::vector<std::vector<int>> swap_before;
        double makespan = 0.0;
        int swap_count = 0;
    };

    struct FullSolution
    {
        bool feasible = false;

        double objective = 0.0;
        double truck_value = 0.0;
        double drone_value = 0.0;

        std::vector<Node> truck_route;
        std::vector<Node> truck_customers;
        std::vector<Node> drone_customers;

        std::vector<std::vector<Node>> per_drone_customers;

        DroneSchedule drone_schedule;

        SolveStats stats;
    };

} // namespace bbcp
