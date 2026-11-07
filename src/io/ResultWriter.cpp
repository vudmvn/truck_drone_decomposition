#include "bbcp/io/ResultWriter.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace bbcp {

namespace {

void EnsureParent(const std::string& path) {
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
}

std::string JoinNodeList(const std::vector<Node>& nodes) {
    std::ostringstream oss;

    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i > 0) {
            oss << " ";
        }
        oss << nodes[i];
    }

    return oss.str();
}

std::string JoinRoute(const std::vector<Node>& route) {
    std::ostringstream oss;

    for (size_t i = 0; i < route.size(); ++i) {
        if (i > 0) {
            oss << " -> ";
        }
        oss << route[i];
    }

    return oss.str();
}

std::string DroneScheduleString(const FullSolution& solution) {
    std::ostringstream oss;

    const auto& chains = solution.drone_schedule.chains;
    const auto& swaps = solution.drone_schedule.swap_before;

    for (size_t k = 0; k < chains.size(); ++k) {
        if (k > 0) {
            oss << " || ";
        }

        oss << "Drone " << k << ": ";

        if (chains[k].empty()) {
            oss << "(not used)";
            continue;
        }

        for (size_t p = 0; p < chains[k].size(); ++p) {
            const bool has_swap =
                k < swaps.size() &&
                p < swaps[k].size() &&
                swaps[k][p] != 0;

            if (p > 0) {
                oss << " | ";
            }

            if (has_swap) {
                oss << "[SWAP] ";
            }

            oss << "0 -> " << chains[k][p] << " -> 0";
        }
    }

    return oss.str();
}

double BestAvailableLowerBound(const FullSolution& solution) {
    double best = -std::numeric_limits<double>::infinity();

    const double final_lb = solution.stats.lower_bound;
    const double tracked_lb = solution.stats.best_master_lb_seen;

    if (std::isfinite(final_lb) && final_lb >= 0.0) {
        best = std::max(best, final_lb);
    }

    if (std::isfinite(tracked_lb) && tracked_lb >= 0.0) {
        best = std::max(best, tracked_lb);
    }

    if (!std::isfinite(best)) {
        return -1.0;
    }

    return best;
}

double SafeGap(double ub, double lb) {
    if (!std::isfinite(ub) || !std::isfinite(lb) || ub < 0.0 || lb < 0.0) {
        return -1.0;
    }

    return std::max(0.0, ub - lb);
}

double SafeGapPercent(double ub, double lb) {
    if (!std::isfinite(ub) || !std::isfinite(lb) || ub <= 1e-9 || lb < 0.0) {
        return -1.0;
    }

    return 100.0 * std::max(0.0, ub - lb) / ub;
}

void WriteSolutionTextToStream(
    std::ostream& out,
    const Instance* inst,
    const FullSolution& solution
) {
    const double best_upper_bound = solution.stats.best_valid_ub;
    const double best_lower_bound = BestAvailableLowerBound(solution);
    const double final_gap = SafeGap(best_upper_bound, best_lower_bound);
    const double final_gap_percent = SafeGapPercent(best_upper_bound, best_lower_bound);

    out << std::fixed << std::setprecision(6);

    out << "\n=== Dataset parameters ===\n";
    if (inst != nullptr) {
        out << "instance: " << inst->name << "\n";
        out << "n_customers: " << inst->n << "\n";
        out << "n_drones: " << inst->n_drones << "\n";
        out << "swap_time_min: " << static_cast<double>(inst->swap_time) / 60.0 << "\n";
        out << "battery_capacity: " << inst->battery_capacity << "\n";
        out << "drone_cap: " << inst->drone_cap << "\n";
        out << "truck_speed: " << inst->truck_speed << "\n";
        out << "drone_speed: " << inst->drone_speed << "\n";
    } else {
        out << "instance: N/A\n";
    }

    out << "\n=== Best solution summary ===\n";
    out << "feasible: " << solution.feasible << "\n";
    out << "best_upper_bound: " << best_upper_bound << "\n";
    out << "best_lower_bound: " << best_lower_bound << "\n";
    out << "final_gap: " << final_gap << "\n";
    out << "final_gap_percent: " << final_gap_percent << "\n";
    out << "truck_value: " << solution.truck_value << "\n";
    out << "drone_value: " << solution.drone_value << "\n";
    out << "final_makespan: "
        << std::max(solution.truck_value, solution.drone_value) << "\n";
    out << "initial_solution: " << solution.stats.initial_valid_ub << "\n";
    out << "time_to_best_solution_sec: "
        << solution.stats.time_to_best_solution_sec << "\n";
    out << "runtime_sec: " << solution.stats.runtime_sec << "\n";
    out << "valid_incumbent_updates: "
        << solution.stats.valid_incumbent_updates << "\n";

    out << "\n=== Customer assignment ===\n";
    out << "num_truck_customers: " << solution.truck_customers.size() << "\n";
    out << "num_drone_customers: " << solution.drone_customers.size() << "\n";
    out << "truck_customers: " << JoinNodeList(solution.truck_customers) << "\n";
    out << "drone_customers: " << JoinNodeList(solution.drone_customers) << "\n";

    out << "\n=== Routes ===\n";
    out << "truck_route: " << JoinRoute(solution.truck_route) << "\n";
    out << "drone_routes: " << DroneScheduleString(solution) << "\n";

    out << "\nper_drone_customers:\n";
    for (size_t k = 0; k < solution.per_drone_customers.size(); ++k) {
        out << "  drone " << k << ": " << JoinNodeList(solution.per_drone_customers[k]) << "\n";
    }

    out << "\n=== Solver diagnostics ===\n";
    out << "gurobi_status: " << solution.stats.gurobi_status << "\n";
    out << "gurobi_master_obj: " << solution.stats.master_obj << "\n";
    out << "gurobi_final_master_lb: " << solution.stats.lower_bound << "\n";
    out << "best_tracked_master_lb: " << solution.stats.best_master_lb_seen << "\n";
    out << "gurobi_master_gap: " << solution.stats.mip_gap << "\n";
    out << "time_to_1pct_gap_sec: "
        << solution.stats.time_to_1pct_gap_sec << "\n";
    out << "first_1pct_gap_value: "
        << solution.stats.first_1pct_gap_value << "\n";
    out << "first_1pct_gap_percent: "
        << solution.stats.first_1pct_gap_percent << "\n";
    out << "first_1pct_gap_bound: "
        << solution.stats.first_1pct_gap_bound << "\n";
    out << "===========================\n";
}

}  // namespace

void ResultWriter::WriteCSV(
    const std::string& path,
    const Instance& inst,
    const MasterOptions& options,
    const FullSolution& solution
) {
    EnsureParent(path);

    const bool need_header =
        !std::filesystem::exists(path) ||
        std::filesystem::file_size(path) == 0;

    std::ofstream out(path, std::ios::app);
    if (!out) {
        std::cerr << "[ERROR] Cannot open CSV file: " << path << "\n";
        return;
    }

    const double best_upper_bound = solution.stats.best_valid_ub;
    const double best_lower_bound = BestAvailableLowerBound(solution);
    const double final_gap = SafeGap(best_upper_bound, best_lower_bound);
    const double final_gap_percent = SafeGapPercent(best_upper_bound, best_lower_bound);

    out << std::fixed << std::setprecision(6);

    if (need_header) {
        out << "instance,"
            << "n_customers,"
            << "n_drones,"
            << "swap_time_min,"
            << "battery_capacity,"
            << "drone_cap,"
            << "truck_speed,"
            << "drone_speed,"

            << "slave,"
            << "feasible,"
            << "best_upper_bound,"
            << "best_lower_bound,"
            << "final_gap,"
            << "final_gap_percent,"
            << "truck_value,"
            << "drone_value,"
            << "initial_solution,"
            << "time_to_best_solution_sec,"
            << "runtime_sec,"

            << "num_truck_customers,"
            << "num_drone_customers,"
            << "truck_customers,"
            << "drone_customers,"
            << "truck_route,"
            << "drone_routes,"

            << "gurobi_status,"
            << "gurobi_master_obj,"
            << "gurobi_final_master_lb,"
            << "best_tracked_master_lb,"
            << "gurobi_master_gap,"
            << "valid_incumbent_updates"
            << "\n";
    }

    out << std::quoted(inst.name) << ","
        << inst.n << ","
        << inst.n_drones << ","
        << static_cast<double>(inst.swap_time) / 60.0 << ","
        << inst.battery_capacity << ","
        << inst.drone_cap << ","
        << inst.truck_speed << ","
        << inst.drone_speed << ","

        << std::quoted(SlaveSolverTypeName(options.slave_solver)) << ","
        << solution.feasible << ","
        << best_upper_bound << ","
        << best_lower_bound << ","
        << final_gap << ","
        << final_gap_percent << ","
        << solution.truck_value << ","
        << solution.drone_value << ","
        << solution.stats.initial_valid_ub << ","
        << solution.stats.time_to_best_solution_sec << ","
        << solution.stats.runtime_sec << ","

        << solution.truck_customers.size() << ","
        << solution.drone_customers.size() << ","
        << std::quoted(JoinNodeList(solution.truck_customers)) << ","
        << std::quoted(JoinNodeList(solution.drone_customers)) << ","
        << std::quoted(JoinRoute(solution.truck_route)) << ","
        << std::quoted(DroneScheduleString(solution)) << ","

        << solution.stats.gurobi_status << ","
        << solution.stats.master_obj << ","
        << solution.stats.lower_bound << ","
        << solution.stats.best_master_lb_seen << ","
        << solution.stats.mip_gap << ","
        << solution.stats.valid_incumbent_updates
        << "\n";
}

void ResultWriter::WriteSolutionText(
    const std::string& path,
    const FullSolution& solution
) {
    EnsureParent(path);

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[ERROR] Cannot open solution text file: " << path << "\n";
        return;
    }

    WriteSolutionTextToStream(out, nullptr, solution);
}

void ResultWriter::WriteSolutionText(
    const std::string& path,
    const Instance& inst,
    const FullSolution& solution
) {
    EnsureParent(path);

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[ERROR] Cannot open solution text file: " << path << "\n";
        return;
    }

    WriteSolutionTextToStream(out, &inst, solution);
}

void ResultWriter::PrintSolutionText(
    const FullSolution& solution
) {
    WriteSolutionTextToStream(std::cout, nullptr, solution);
}

}  // namespace bbcp
