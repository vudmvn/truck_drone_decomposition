#!/usr/bin/env python3
"""SQLite Database Manager for Truck-Drone Decomposition Solver Outputs.

Provides CLI commands to initialize schema, import historical CSV results,
query experiment summaries, show run details, and run custom queries.
"""

import argparse
import csv
import os
import sqlite3
import sys
from pathlib import Path
from typing import Dict, Any, List, Optional

DEFAULT_DB = str(Path(__file__).resolve().parent.parent / "results" / "results.db")

SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS experiments (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    instance TEXT NOT NULL,
    n_customers INTEGER,
    n_drones INTEGER,
    swap_time_min REAL,
    battery_capacity REAL,
    drone_cap REAL,
    truck_speed REAL,
    drone_speed REAL,
    assignment_mode TEXT,
    slave TEXT,
    truck_mode TEXT,
    branching TEXT,
    feasible INTEGER,
    best_upper_bound REAL,
    best_lower_bound REAL,
    final_gap REAL,
    final_gap_percent REAL,
    truck_value REAL,
    drone_value REAL,
    final_makespan REAL,
    initial_solution REAL,
    time_to_best_solution_sec REAL,
    runtime_sec REAL,
    num_truck_customers INTEGER,
    num_drone_customers INTEGER,
    truck_customers TEXT,
    drone_customers TEXT,
    truck_route TEXT,
    drone_routes TEXT,
    gurobi_status INTEGER,
    gurobi_master_obj REAL,
    gurobi_final_master_lb REAL,
    best_tracked_master_lb REAL,
    gurobi_master_gap REAL,
    valid_incumbent_updates INTEGER,
    mipnode_cuts INTEGER,
    local_search INTEGER
);

CREATE INDEX IF NOT EXISTS idx_experiments_instance ON experiments(instance);
CREATE INDEX IF NOT EXISTS idx_experiments_feasible ON experiments(feasible);
CREATE INDEX IF NOT EXISTS idx_experiments_slave ON experiments(slave);
CREATE INDEX IF NOT EXISTS idx_experiments_assignment ON experiments(assignment_mode);

CREATE VIEW IF NOT EXISTS v_summary AS
SELECT 
    id,
    timestamp,
    instance,
    n_customers,
    n_drones,
    assignment_mode,
    slave,
    feasible,
    best_upper_bound AS UB,
    best_lower_bound AS LB,
    final_gap_percent AS gap_pct,
    runtime_sec,
    gurobi_status
FROM experiments;
"""


def get_connection(db_path: str) -> sqlite3.Connection:
    Path(db_path).parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    return conn


def init_db(db_path: str) -> None:
    conn = get_connection(db_path)
    with conn:
        conn.executescript(SCHEMA_SQL)
    conn.close()
    print(f"[OK] Initialized SQLite schema in: {db_path}")


def to_float(val: Any) -> Optional[float]:
    if val is None:
        return None
    s = str(val).strip()
    if not s or s.lower() in ("inf", "-inf", "nan", "n/a", "none"):
        return None
    try:
        f = float(s)
        return None if f < -1e8 else f
    except ValueError:
        return None


def to_int(val: Any) -> Optional[int]:
    if val is None:
        return None
    s = str(val).strip()
    if not s or s.lower() in ("inf", "-inf", "nan", "n/a", "none"):
        return None
    try:
        return int(float(s))
    except ValueError:
        return None


def import_csv_row(conn: sqlite3.Connection, row: Dict[str, str], default_assignment: str = "") -> bool:
    # Handle different column namings across CSV variants
    inst = row.get("instance", "").strip('"\'')
    if not inst:
        return False

    n_cust = to_int(row.get("n_customers", row.get("n")))
    n_drones = to_int(row.get("n_drones"))
    swap_min = to_float(row.get("swap_time_min", 0.0))
    bat_cap = to_float(row.get("battery_capacity", 1000.0))
    drone_cap = to_float(row.get("drone_cap", 2.27))
    truck_speed = to_float(row.get("truck_speed", 30.0))
    drone_speed = to_float(row.get("drone_speed", 40.0))

    slave = row.get("slave", "").strip('"\'')
    feasible = to_int(row.get("feasible", 0))

    ub = to_float(row.get("best_upper_bound", row.get("best_valid_ub", row.get("obj"))))
    lb = to_float(row.get("best_lower_bound", row.get("best_master_lb_seen", row.get("lower_bound"))))

    final_gap = to_float(row.get("final_gap", row.get("final_valid_gap")))
    final_gap_pct = to_float(row.get("final_gap_percent", row.get("final_valid_gap_percent", row.get("mip_gap"))))

    truck_val = to_float(row.get("truck_value", row.get("truck")))
    drone_val = to_float(row.get("drone_value", row.get("drone")))
    makespan = max(truck_val or 0.0, drone_val or 0.0) if (truck_val is not None or drone_val is not None) else ub

    initial_sol = to_float(row.get("initial_solution", row.get("initial_valid_ub")))
    time_to_best = to_float(row.get("time_to_best_solution_sec"))
    runtime_sec = to_float(row.get("runtime_sec"))

    num_truck = to_int(row.get("num_truck_customers"))
    num_drone = to_int(row.get("num_drone_customers"))
    truck_cust = row.get("truck_customers", "").strip('"\'')
    drone_cust = row.get("drone_customers", "").strip('"\'')
    truck_route = row.get("truck_route", "").strip('"\'')
    drone_routes = row.get("drone_routes", row.get("drone_schedules", "")).strip('"\'')

    status = to_int(row.get("gurobi_status"))
    master_obj = to_float(row.get("gurobi_master_obj", row.get("master_obj")))
    final_lb = to_float(row.get("gurobi_final_master_lb", row.get("lower_bound")))
    tracked_lb = to_float(row.get("best_tracked_master_lb", row.get("best_master_lb_seen")))
    master_gap = to_float(row.get("gurobi_master_gap", row.get("mip_gap")))
    valid_updates = to_int(row.get("valid_incumbent_updates", 0))

    # Detect assignment mode from path or argument
    assignment_mode = default_assignment
    if not assignment_mode:
        if "slave_assign" in row.get("file_source", ""):
            assignment_mode = "slave-assign"
        elif "explicit" in row.get("file_source", ""):
            assignment_mode = "explicit"
        else:
            assignment_mode = "explicit"

    sql = """
    INSERT INTO experiments (
        instance, n_customers, n_drones, swap_time_min, battery_capacity, drone_cap,
        truck_speed, drone_speed, assignment_mode, slave, truck_mode, branching,
        feasible, best_upper_bound, best_lower_bound, final_gap, final_gap_percent,
        truck_value, drone_value, final_makespan, initial_solution,
        time_to_best_solution_sec, runtime_sec, num_truck_customers, num_drone_customers,
        truck_customers, drone_customers, truck_route, drone_routes,
        gurobi_status, gurobi_master_obj, gurobi_final_master_lb, best_tracked_master_lb,
        gurobi_master_gap, valid_incumbent_updates, mipnode_cuts, local_search
    ) VALUES (
        ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
        ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
        ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?
    );
    """
    conn.execute(sql, (
        inst, n_cust, n_drones, swap_min, bat_cap, drone_cap,
        truck_speed, drone_speed, assignment_mode, slave, "sec", "hybrid",
        feasible, ub, lb, final_gap, final_gap_pct,
        truck_val, drone_val, makespan, initial_sol,
        time_to_best, runtime_sec, num_truck, num_drone,
        truck_cust, drone_cust, truck_route, drone_routes,
        status, master_obj, final_lb, tracked_lb,
        master_gap, valid_updates, 0, 0
    ))
    return True


def import_csv_file(db_path: str, csv_path: str) -> int:
    init_db(db_path)
    conn = get_connection(db_path)
    imported = 0
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        with conn:
            for row in reader:
                row["file_source"] = csv_path
                if import_csv_row(conn, row):
                    imported += 1
    conn.close()
    print(f"[OK] Imported {imported} rows from {csv_path} into {db_path}")
    return imported


def import_all_csvs(db_path: str, results_dir: str = "results") -> int:
    init_db(db_path)
    total = 0
    for root, _, files in os.walk(results_dir):
        for file in sorted(files):
            if file.endswith(".csv"):
                csv_path = os.path.join(root, file)
                total += import_csv_file(db_path, csv_path)
    print(f"[SUMMARY] Total {total} rows imported into {db_path}")
    return total


def list_runs(db_path: str, limit: int = 50) -> None:
    if not os.path.exists(db_path):
        print(f"[WARN] Database does not exist: {db_path}")
        return
    conn = get_connection(db_path)
    cur = conn.cursor()
    cur.execute("""
        SELECT id, timestamp, instance, n_customers, n_drones, assignment_mode, slave,
               feasible, best_upper_bound, best_lower_bound, final_gap_percent, runtime_sec
        FROM experiments
        ORDER BY id DESC
        LIMIT ?;
    """, (limit,))
    rows = cur.fetchall()
    conn.close()

    if not rows:
        print("[INFO] No experiment records found.")
        return

    print("-" * 115)
    print(f"{'ID':<4} | {'Timestamp':<19} | {'Instance':<30} | {'K':<2} | {'Assign':<11} | {'Slave':<5} | {'Feas':<4} | {'UB':<8} | {'LB':<8} | {'Gap %':<7} | {'Time(s)':<7}")
    print("-" * 115)
    for r in rows:
        ub_str = f"{r['best_upper_bound']:.1f}" if r['best_upper_bound'] is not None else "N/A"
        lb_str = f"{r['best_lower_bound']:.1f}" if r['best_lower_bound'] is not None else "N/A"
        gap_str = f"{r['final_gap_percent']:.2f}%" if r['final_gap_percent'] is not None else "N/A"
        time_str = f"{r['runtime_sec']:.2f}" if r['runtime_sec'] is not None else "N/A"
        inst_name = Path(r['instance']).name if r['instance'] else "N/A"
        print(f"{r['id']:<4} | {str(r['timestamp'])[:19]:<19} | {inst_name[:30]:<30} | {r['n_drones'] or '-':<2} | {str(r['assignment_mode'] or '-'):<11} | {str(r['slave'] or '-'):<5} | {r['feasible'] or 0:<4} | {ub_str:<8} | {lb_str:<8} | {gap_str:<7} | {time_str:<7}")
    print("-" * 115)


def show_run(db_path: str, run_id: int) -> None:
    if not os.path.exists(db_path):
        print(f"[WARN] Database does not exist: {db_path}")
        return
    conn = get_connection(db_path)
    cur = conn.cursor()
    cur.execute("SELECT * FROM experiments WHERE id = ?;", (run_id,))
    row = cur.fetchone()
    conn.close()

    if not row:
        print(f"[ERROR] Run id {run_id} not found.")
        return

    print(f"\n=== Experiment Run Details [ID: {run_id}] ===")
    for key in row.keys():
        print(f"  {key:<28}: {row[key]}")
    print("=" * 45)


def export_csv(db_path: str, output_csv: str) -> None:
    if not os.path.exists(db_path):
        print(f"[WARN] Database does not exist: {db_path}")
        return
    conn = get_connection(db_path)
    cur = conn.cursor()
    cur.execute("SELECT * FROM experiments ORDER BY id ASC;")
    rows = cur.fetchall()
    conn.close()

    if not rows:
        print("[INFO] No records to export.")
        return

    Path(output_csv).parent.mkdir(parents=True, exist_ok=True)
    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(rows[0].keys())
        for row in rows:
            writer.writerow(list(row))
    print(f"[OK] Exported {len(rows)} rows to {output_csv}")


def main():
    parser = argparse.ArgumentParser(description="Manage SQLite outputs database for Truck-Drone Solver.")
    parser.add_argument("--db", default=DEFAULT_DB, help=f"Path to SQLite database (default: {DEFAULT_DB})")

    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("init", help="Initialize database schema and views")

    import_csv_parser = subparsers.add_parser("import-csv", help="Import a single CSV file")
    import_csv_parser.add_argument("csv_file", help="Path to CSV file")

    import_all_parser = subparsers.add_parser("import-all", help="Scan and import all CSV files under results directory")
    import_all_parser.add_argument("--dir", default="results", help="Directory containing CSV files (default: results)")

    list_parser = subparsers.add_parser("list", help="List recent experiment runs")
    list_parser.add_argument("--limit", type=int, default=50, help="Max records to display")

    show_parser = subparsers.add_parser("show", help="Show full details of a specific run")
    show_parser.add_argument("id", type=int, help="Run ID")

    export_parser = subparsers.add_parser("export-csv", help="Export all runs from database to CSV")
    export_parser.add_argument("output_csv", help="Path to output CSV file")

    query_parser = subparsers.add_parser("sql", help="Execute arbitrary SQL query")
    query_parser.add_argument("query", help="SQL SELECT query")

    args = parser.parse_args()

    if not args.command or args.command == "list":
        list_runs(args.db, getattr(args, "limit", 50))
    elif args.command == "init":
        init_db(args.db)
    elif args.command == "import-csv":
        import_csv_file(args.db, args.csv_file)
    elif args.command == "import-all":
        import_all_csvs(args.db, args.dir)
    elif args.command == "show":
        show_run(args.db, args.id)
    elif args.command == "export-csv":
        export_csv(args.db, args.output_csv)
    elif args.command == "sql":
        conn = get_connection(args.db)
        cur = conn.cursor()
        cur.execute(args.query)
        rows = cur.fetchall()
        if rows:
            headers = [d[0] for d in cur.description]
            print("\t".join(headers))
            for r in rows:
                print("\t".join(str(r[h]) for h in headers))
        else:
            print("[INFO] Query returned 0 rows.")
        conn.close()


if __name__ == "__main__":
    main()
