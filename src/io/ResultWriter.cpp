#include "bbcp/io/ResultWriter.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <sqlite3.h>

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

void ResultWriter::WriteSQLite(
    const std::string& db_path,
    const Instance& inst,
    const MasterOptions& options,
    const FullSolution& solution
) {
    if (db_path.empty()) return;

    EnsureParent(db_path);

    sqlite3* db = nullptr;
    const int rc = sqlite3_open_v2(
        db_path.c_str(),
        &db,
        SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE,
        nullptr
    );

    if (rc != SQLITE_OK) {
        std::cerr << "[ERROR] Cannot open SQLite database: " << db_path
                  << " (" << sqlite3_errmsg(db) << ")\n";
        if (db) sqlite3_close(db);
        return;
    }

    sqlite3_busy_timeout(db, 5000);

    const char* schema_sql =
        "CREATE TABLE IF NOT EXISTS experiments (\n"
        "    id INTEGER PRIMARY KEY AUTOINCREMENT,\n"
        "    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,\n"
        "    instance TEXT NOT NULL,\n"
        "    n_customers INTEGER,\n"
        "    n_drones INTEGER,\n"
        "    swap_time_min REAL,\n"
        "    battery_capacity REAL,\n"
        "    drone_cap REAL,\n"
        "    truck_speed REAL,\n"
        "    drone_speed REAL,\n"
        "    assignment_mode TEXT,\n"
        "    slave TEXT,\n"
        "    truck_mode TEXT,\n"
        "    branching TEXT,\n"
        "    feasible INTEGER,\n"
        "    best_upper_bound REAL,\n"
        "    best_lower_bound REAL,\n"
        "    final_gap REAL,\n"
        "    final_gap_percent REAL,\n"
        "    truck_value REAL,\n"
        "    drone_value REAL,\n"
        "    final_makespan REAL,\n"
        "    initial_solution REAL,\n"
        "    time_to_best_solution_sec REAL,\n"
        "    runtime_sec REAL,\n"
        "    num_truck_customers INTEGER,\n"
        "    num_drone_customers INTEGER,\n"
        "    truck_customers TEXT,\n"
        "    drone_customers TEXT,\n"
        "    truck_route TEXT,\n"
        "    drone_routes TEXT,\n"
        "    gurobi_status INTEGER,\n"
        "    gurobi_master_obj REAL,\n"
        "    gurobi_final_master_lb REAL,\n"
        "    best_tracked_master_lb REAL,\n"
        "    gurobi_master_gap REAL,\n"
        "    valid_incumbent_updates INTEGER,\n"
        "    mipnode_cuts INTEGER,\n"
        "    local_search INTEGER\n"
        ");\n"
        "CREATE INDEX IF NOT EXISTS idx_experiments_instance ON experiments(instance);\n"
        "CREATE INDEX IF NOT EXISTS idx_experiments_feasible ON experiments(feasible);\n"
        "CREATE INDEX IF NOT EXISTS idx_experiments_slave ON experiments(slave);\n"
        "CREATE INDEX IF NOT EXISTS idx_experiments_assignment ON experiments(assignment_mode);\n"
        "CREATE VIEW IF NOT EXISTS v_summary AS\n"
        "SELECT \n"
        "    id,\n"
        "    timestamp,\n"
        "    instance,\n"
        "    n_customers,\n"
        "    n_drones,\n"
        "    assignment_mode,\n"
        "    slave,\n"
        "    feasible,\n"
        "    best_upper_bound AS UB,\n"
        "    best_lower_bound AS LB,\n"
        "    final_gap_percent AS gap_pct,\n"
        "    runtime_sec,\n"
        "    gurobi_status\n"
        "FROM experiments;\n";

    char* err_msg = nullptr;
    if (sqlite3_exec(db, schema_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
        std::cerr << "[ERROR] SQLite schema initialization error: "
                  << (err_msg ? err_msg : "unknown") << "\n";
        sqlite3_free(err_msg);
        sqlite3_close(db);
        return;
    }

    const char* insert_sql =
        "INSERT INTO experiments (\n"
        "    instance, n_customers, n_drones, swap_time_min, battery_capacity, drone_cap,\n"
        "    truck_speed, drone_speed, assignment_mode, slave, truck_mode, branching,\n"
        "    feasible, best_upper_bound, best_lower_bound, final_gap, final_gap_percent,\n"
        "    truck_value, drone_value, final_makespan, initial_solution,\n"
        "    time_to_best_solution_sec, runtime_sec, num_truck_customers, num_drone_customers,\n"
        "    truck_customers, drone_customers, truck_route, drone_routes,\n"
        "    gurobi_status, gurobi_master_obj, gurobi_final_master_lb, best_tracked_master_lb,\n"
        "    gurobi_master_gap, valid_incumbent_updates, mipnode_cuts, local_search\n"
        ") VALUES (\n"
        "    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,\n"
        "    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,\n"
        "    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?\n"
        ");";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, insert_sql, -1, &stmt, nullptr) != SQLITE_OK) {
        std::cerr << "[ERROR] Cannot prepare SQLite insert: "
                  << sqlite3_errmsg(db) << "\n";
        sqlite3_close(db);
        return;
    }

    const double best_upper_bound = solution.stats.best_valid_ub;
    const double best_lower_bound = BestAvailableLowerBound(solution);
    const double final_gap = SafeGap(best_upper_bound, best_lower_bound);
    const double final_gap_percent = SafeGapPercent(best_upper_bound, best_lower_bound);
    const double final_makespan = std::max(solution.truck_value, solution.drone_value);

    auto bind_double_safe = [](sqlite3_stmt* s, int idx, double val) {
        if (!std::isfinite(val)) {
            sqlite3_bind_null(s, idx);
        } else {
            sqlite3_bind_double(s, idx, val);
        }
    };

    const std::string instance_name = inst.name;
    const std::string assignment_str = MasterAssignmentModeName(options.assignment_mode);
    const std::string slave_str = SlaveSolverTypeName(options.slave_solver);
    const std::string truck_mode_str = MasterSubtourModeName(options.subtour_mode);
    const std::string branching_str = BranchingStrategyName(options.branching_strategy);
    const std::string truck_cust_str = JoinNodeList(solution.truck_customers);
    const std::string drone_cust_str = JoinNodeList(solution.drone_customers);
    const std::string truck_route_str = JoinRoute(solution.truck_route);
    const std::string drone_routes_str = DroneScheduleString(solution);

    int p = 1;
    sqlite3_bind_text(stmt, p++, instance_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, p++, inst.n);
    sqlite3_bind_int(stmt, p++, inst.n_drones);
    bind_double_safe(stmt, p++, static_cast<double>(inst.swap_time) / 60.0);
    bind_double_safe(stmt, p++, inst.battery_capacity);
    bind_double_safe(stmt, p++, inst.drone_cap);
    bind_double_safe(stmt, p++, inst.truck_speed);
    bind_double_safe(stmt, p++, inst.drone_speed);
    sqlite3_bind_text(stmt, p++, assignment_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, p++, slave_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, p++, truck_mode_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, p++, branching_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, p++, solution.feasible ? 1 : 0);
    bind_double_safe(stmt, p++, best_upper_bound);
    bind_double_safe(stmt, p++, best_lower_bound);
    bind_double_safe(stmt, p++, final_gap);
    bind_double_safe(stmt, p++, final_gap_percent);
    bind_double_safe(stmt, p++, solution.truck_value);
    bind_double_safe(stmt, p++, solution.drone_value);
    bind_double_safe(stmt, p++, final_makespan);
    bind_double_safe(stmt, p++, solution.stats.initial_valid_ub);
    bind_double_safe(stmt, p++, solution.stats.time_to_best_solution_sec);
    bind_double_safe(stmt, p++, solution.stats.runtime_sec);
    sqlite3_bind_int(stmt, p++, static_cast<int>(solution.truck_customers.size()));
    sqlite3_bind_int(stmt, p++, static_cast<int>(solution.drone_customers.size()));
    sqlite3_bind_text(stmt, p++, truck_cust_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, p++, drone_cust_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, p++, truck_route_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, p++, drone_routes_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, p++, solution.stats.gurobi_status);
    bind_double_safe(stmt, p++, solution.stats.master_obj);
    bind_double_safe(stmt, p++, solution.stats.lower_bound);
    bind_double_safe(stmt, p++, solution.stats.best_master_lb_seen);
    bind_double_safe(stmt, p++, solution.stats.mip_gap);
    sqlite3_bind_int(stmt, p++, solution.stats.valid_incumbent_updates);
    sqlite3_bind_int(stmt, p++, options.enable_mipnode_cuts ? 1 : 0);
    sqlite3_bind_int(stmt, p++, options.local_search_options.enabled ? 1 : 0);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
        std::cerr << "[ERROR] Failed to insert row into SQLite: "
                  << sqlite3_errmsg(db) << "\n";
    } else if (options.verbose) {
        std::cout << "[SQLITE] Output stored in: " << db_path
                  << " (rowid=" << sqlite3_last_insert_rowid(db) << ")\n";
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
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
