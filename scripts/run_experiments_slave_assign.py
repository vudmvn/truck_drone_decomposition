#!/usr/bin/env python3
"""Batch experiment runner for Truck-Drone Decomposition Solver.

Runs instances of sizes n=30, n=50, and n=100 with configuration:
- Master: MILP (Gurobi) with SEC lazy subtour elimination constraints
- Slave: CP-SAT (OR-Tools)
- Assignment Mode: slave-assign
- Branching: hybrid
- Time Limit: up to 3600 seconds per instance
- Storage: CSV & SQLite database (results/results.db)
"""

import argparse
import datetime
import json
import os
import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple, Set

DEFAULT_BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_DATA_DIR = DEFAULT_BASE_DIR / "data" / "instances"
DEFAULT_BIN = DEFAULT_BASE_DIR / "build" / "truck_drone_solver"
DEFAULT_DB = DEFAULT_BASE_DIR / "results" / "results.db"
DEFAULT_CSV = DEFAULT_BASE_DIR / "results" / "csv" / "result_slave_assign.csv"
DEFAULT_LOG_DIR = DEFAULT_BASE_DIR / "results" / "logs" / "slave_assign"

current_child_proc: Optional[subprocess.Popen] = None


def sig_handler(signum, frame):
    global current_child_proc
    print(f"\n[INTERRUPT] Received signal {signum}. Terminating current solver process gracefully...", flush=True)
    if current_child_proc and current_child_proc.poll() is None:
        try:
            current_child_proc.terminate()
            current_child_proc.wait(timeout=5)
        except Exception:
            current_child_proc.kill()
    sys.exit(128 + signum)


signal.signal(signal.SIGINT, sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


def get_completed_instances(db_path: Path, assignment_mode: str, slave: str) -> Set[str]:
    """Retrieve set of instance names already completed in database."""
    completed = set()
    if not db_path.is_file():
        return completed

    try:
        conn = sqlite3.connect(str(db_path))
        cur = conn.cursor()
        cur.execute(
            """
            SELECT DISTINCT instance
            FROM experiments
            WHERE assignment_mode = ? AND slave = ?;
            """,
            (assignment_mode, slave),
        )
        for row in cur.fetchall():
            completed.add(row[0])
            completed.add(Path(row[0]).name)
        conn.close()
    except Exception as e:
        print(f"[WARN] Failed to query completed instances from {db_path}: {e}", file=sys.stderr)

    return completed


def collect_instances(data_dir: Path, sizes: List[int]) -> List[Tuple[int, Path]]:
    """Collect instance files grouped and ordered by customer size."""
    instances = []
    for sz in sizes:
        pattern = f"{sz}-*.txt"
        files = sorted(list(data_dir.glob(pattern)))
        for f in files:
            instances.append((sz, f))
    return instances


def main():
    parser = argparse.ArgumentParser(
        description="Run batch experiments for truck-drone decomposition solver (slave-assign + CP)."
    )
    parser.add_argument("--bin", type=Path, default=DEFAULT_BIN, help=f"Solver binary (default: {DEFAULT_BIN})")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR, help="Instances directory")
    parser.add_argument("--db", type=Path, default=DEFAULT_DB, help="SQLite database path")
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="CSV output path")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="Logs directory")
    parser.add_argument("--time-limit", type=int, default=3600, help="Master solver time limit in seconds (default: 3600)")
    parser.add_argument("--slave-time", type=int, default=30, help="Slave solver time limit in seconds (default: 30)")
    parser.add_argument("--truck", default="sec", choices=["sec", "mtz"], help="Truck subtour mode (default: sec)")
    parser.add_argument("--branching", default="hybrid", help="Branching strategy (default: hybrid)")
    parser.add_argument("--assignment", default="slave-assign", help="Assignment mode (default: slave-assign)")
    parser.add_argument("--slave", default="cp", help="Slave solver (default: cp)")
    parser.add_argument("--sizes", default="30,50,100", help="Comma-separated instance sizes to run (default: 30,50,100)")
    parser.add_argument("--force", action="store_true", help="Force rerun even if instance already in DB")
    parser.add_argument("--dry-run", action="store_true", help="List instances without executing")

    args = parser.parse_args()

    if not args.bin.is_file():
        print(f"[ERROR] Solver binary not found: {args.bin}", file=sys.stderr)
        print("Run `cmake --build build -j` to compile it first.", file=sys.stderr)
        sys.exit(1)

    if not args.data_dir.is_dir():
        print(f"[ERROR] Data directory not found: {args.data_dir}", file=sys.stderr)
        sys.exit(1)

    args.log_dir.mkdir(parents=True, exist_ok=True)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    args.db.parent.mkdir(parents=True, exist_ok=True)

    sizes = [int(s.strip()) for s in args.sizes.split(",") if s.strip()]
    instances = collect_instances(args.data_dir, sizes)

    if not instances:
        print(f"[ERROR] No instances found in {args.data_dir} for sizes {sizes}", file=sys.stderr)
        sys.exit(1)

    completed = set() if args.force else get_completed_instances(args.db, args.assignment, args.slave)

    total_count = len(instances)
    run_list = [(sz, p) for (sz, p) in instances if (p.name not in completed and str(p) not in completed)]
    skipped_count = total_count - len(run_list)

    print("=" * 80)
    print("TRUCK-DRONE DECOMPOSITION BATCH RUNNER")
    print(f"Timestamp        : {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Solver binary    : {args.bin}")
    print(f"Assignment mode  : {args.assignment}")
    print(f"Slave solver     : {args.slave}")
    print(f"Truck mode       : {args.truck}")
    print(f"Branching        : {args.branching}")
    print(f"Master time limit: {args.time_limit}s")
    print(f"Slave time limit : {args.slave_time}s")
    print(f"SQLite DB        : {args.db}")
    print(f"Result CSV       : {args.csv}")
    print(f"Logs dir         : {args.log_dir}")
    print(f"Sizes            : {sizes}")
    print(f"Total instances  : {total_count} (To run: {len(run_list)}, Already completed: {skipped_count})")
    print("=" * 80)

    if args.dry_run:
        print("\n[DRY RUN] Instances scheduled to run:")
        for idx, (sz, p) in enumerate(run_list, 1):
            print(f"  [{idx:3d}/{len(run_list)}] size={sz} file={p.name}")
        return

    progress_file = args.log_dir / "progress.json"

    def save_progress(current_idx: int, current_inst: str, status: str):
        data = {
            "last_update": datetime.datetime.now().isoformat(),
            "total": total_count,
            "to_run": len(run_list),
            "completed_before": skipped_count,
            "current_index": current_idx,
            "current_instance": current_inst,
            "status": status,
        }
        with open(progress_file, "w", encoding="utf-8") as pf:
            json.dump(data, pf, indent=2)

    global current_child_proc
    success_count = 0
    failure_count = 0

    for idx, (sz, inst_path) in enumerate(run_list, 1):
        inst_name = inst_path.name
        log_file = args.log_dir / f"{inst_path.stem}.log"

        print(f"\n>>> [{idx}/{len(run_list)}] [{datetime.datetime.now().strftime('%H:%M:%S')}] Starting {inst_name} (n={sz})...", flush=True)
        save_progress(idx, inst_name, "RUNNING")

        cmd = [
            str(args.bin),
            str(inst_path),
            "--assignment", args.assignment,
            "--slave", args.slave,
            "--truck", args.truck,
            "--branching", args.branching,
            "--time-limit", str(args.time_limit),
            "--slave-time", str(args.slave_time),
            "--result", str(args.csv),
            "--db", str(args.db),
        ]

        t_start = time.time()
        with open(log_file, "w", encoding="utf-8") as lf:
            lf.write(f"# Command: {' '.join(cmd)}\n")
            lf.write(f"# Started: {datetime.datetime.now().isoformat()}\n\n")
            lf.flush()

            try:
                current_child_proc = subprocess.Popen(
                    cmd,
                    stdout=lf,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                retcode = current_child_proc.wait()
                current_child_proc = None
            except Exception as e:
                lf.write(f"\n[EXCEPTION] {e}\n")
                retcode = -1

        elapsed = time.time() - t_start

        # Parse log for result summary
        ub_val = "N/A"
        lb_val = "N/A"
        status_code = retcode
        try:
            with open(log_file, "r", encoding="utf-8", errors="replace") as lf:
                for line in lf:
                    if "[RESULT]" in line:
                        parts = line.strip().split()
                        for p in parts:
                            if p.startswith("UB="): ub_val = p.split("=")[1]
                            if p.startswith("LB="): lb_val = p.split("=")[1]
                            if p.startswith("master_status="): status_code = p.split("=")[1]
        except Exception:
            pass

        if retcode == 0:
            success_count += 1
            print(f"<<< [{idx}/{len(run_list)}] [{datetime.datetime.now().strftime('%H:%M:%S')}] Finished {inst_name} in {elapsed:.1f}s -> Status: {status_code}, UB: {ub_val}, LB: {lb_val}", flush=True)
            save_progress(idx, inst_name, "DONE")
        else:
            failure_count += 1
            print(f"!!! [{idx}/{len(run_list)}] [{datetime.datetime.now().strftime('%H:%M:%S')}] FAILED {inst_name} (retcode={retcode}) in {elapsed:.1f}s -> Log: {log_file}", flush=True)
            save_progress(idx, inst_name, f"ERROR (code {retcode})")

    print("\n" + "=" * 80)
    print("BATCH RUN COMPLETED")
    print(f"Total processed: {len(run_list)}")
    print(f"Success        : {success_count}")
    print(f"Failed         : {failure_count}")
    print(f"Results DB     : {args.db}")
    print(f"Results CSV    : {args.csv}")
    print("=" * 80)


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        sys.exit(0)
