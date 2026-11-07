#!/usr/bin/env python3
"""Batch experiment runner for 2-Index Anonymous CP-SAT Solver with Lipschitz Bounds.

Runs instances of sizes n=30, n=50, and n=100 in parallel (default: 2 jobs).
Configuration:
- Model: Monolithic 2-Index Anonymous CP-SAT (OR-Tools)
- Enhancements: Lipschitz workload cuts, energy cover, symmetry breaking, pair cuts
- Time Limit: up to 3600 seconds per instance
- Storage: CSV & SQLite database (results/results.db)
"""

import argparse
import concurrent.futures
import datetime
import json
import os
import signal
import sqlite3
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Set

DEFAULT_BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_DATA_DIR = DEFAULT_BASE_DIR / "data" / "instances"
DEFAULT_BIN = DEFAULT_BASE_DIR / "build" / "truck_drone_solver"
DEFAULT_DB = DEFAULT_BASE_DIR / "results" / "results.db"
DEFAULT_CSV = DEFAULT_BASE_DIR / "results" / "csv" / "result_cp_2index.csv"
DEFAULT_LOG_DIR = DEFAULT_BASE_DIR / "results" / "logs" / "cp_2index"

lock = threading.Lock()
active_procs: Dict[str, subprocess.Popen] = {}
is_shutting_down = False


def sig_handler(signum, frame):
    global is_shutting_down
    is_shutting_down = True
    print(f"\n[INTERRUPT] Received signal {signum}. Terminating all active solver processes gracefully...", flush=True)
    with lock:
        for name, proc in list(active_procs.items()):
            if proc.poll() is None:
                try:
                    proc.terminate()
                except Exception:
                    pass
        time.sleep(0.5)
        for name, proc in list(active_procs.items()):
            if proc.poll() is None:
                try:
                    proc.kill()
                except Exception:
                    pass
    sys.exit(128 + signum)


signal.signal(signal.SIGINT, sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


def get_completed_instances(db_path: Path) -> Set[str]:
    """Retrieve set of instance names already completed in database with cp-2index."""
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
            WHERE assignment_mode = 'cp-2index';
            """
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
        description="Run parallel batch experiments for 2-Index Anonymous CP-SAT solver with Lipschitz inequalities."
    )
    parser.add_argument("--bin", type=Path, default=DEFAULT_BIN, help=f"Solver binary (default: {DEFAULT_BIN})")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR, help="Instances directory")
    parser.add_argument("--db", type=Path, default=DEFAULT_DB, help="SQLite database path")
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="CSV output path")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="Logs directory")
    parser.add_argument("--time-limit", type=int, default=3600, help="Time limit in seconds (default: 3600)")
    parser.add_argument("-j", "--jobs", "--parallel", dest="jobs", type=int, default=2, help="Number of instances to solve in parallel (default: 2)")
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

    completed = set() if args.force else get_completed_instances(args.db)

    total_count = len(instances)
    run_list = [(sz, p) for (sz, p) in instances if (p.name not in completed and str(p) not in completed)]
    skipped_count = total_count - len(run_list)

    print("=" * 80)
    print("PARALLEL 2-INDEX ANONYMOUS CP-SAT BATCH RUNNER")
    print(f"Timestamp        : {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Solver binary    : {args.bin}")
    print(f"Model            : cp-2index (Monolithic CP-SAT)")
    print(f"Enhancements     : Lipschitz bounds, energy cover, symmetry breaking, pair cuts")
    print(f"Parallel jobs    : {args.jobs} concurrent instances")
    print(f"Time limit       : {args.time_limit}s per instance")
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

    running_set: Set[str] = set()
    success_count = 0
    failure_count = 0
    completed_in_run = 0

    def update_progress_file():
        data = {
            "last_update": datetime.datetime.now().isoformat(),
            "total": total_count,
            "to_run": len(run_list),
            "completed_before": skipped_count,
            "completed_now": completed_in_run,
            "success": success_count,
            "failed": failure_count,
            "parallel_jobs": args.jobs,
            "currently_running": sorted(list(running_set)),
            "status": "RUNNING" if running_set else "FINISHED",
        }
        try:
            with open(progress_file, "w", encoding="utf-8") as pf:
                json.dump(data, pf, indent=2)
        except Exception:
            pass

    def run_one(item: Tuple[int, int, Path]) -> Tuple[str, int, float, str, str]:
        nonlocal success_count, failure_count, completed_in_run
        seq_num, sz, inst_path = item
        inst_name = inst_path.name
        log_file = args.log_dir / f"{inst_path.stem}.log"

        with lock:
            running_set.add(inst_name)
            update_progress_file()
            print(f">>> [START {seq_num}/{len(run_list)}] [{datetime.datetime.now().strftime('%H:%M:%S')}] {inst_name} (n={sz}) [Active: {len(running_set)}]", flush=True)

        cmd = [
            str(args.bin),
            str(inst_path),
            "--model", "cp-2index",
            "--time-limit", str(args.time_limit),
            "--result", str(args.csv),
            "--db", str(args.db),
        ]

        t_start = time.time()
        retcode = -1
        try:
            with open(log_file, "w", encoding="utf-8") as lf:
                lf.write(f"# Command: {' '.join(cmd)}\n")
                lf.write(f"# Started: {datetime.datetime.now().isoformat()}\n\n")
                lf.flush()

                proc = subprocess.Popen(
                    cmd,
                    stdout=lf,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                with lock:
                    active_procs[inst_name] = proc

                retcode = proc.wait()

                with lock:
                    active_procs.pop(inst_name, None)
        except Exception as e:
            with lock:
                active_procs.pop(inst_name, None)
            retcode = -1

        elapsed = time.time() - t_start

        ub_val = "N/A"
        lb_val = "N/A"
        try:
            with open(log_file, "r", encoding="utf-8", errors="replace") as lf:
                for line in lf:
                    if "[RESULT]" in line:
                        parts = line.strip().split()
                        for p in parts:
                            if p.startswith("UB="): ub_val = p.split("=")[1]
                            if p.startswith("LB="): lb_val = p.split("=")[1]
        except Exception:
            pass

        with lock:
            running_set.discard(inst_name)
            completed_in_run += 1
            if retcode == 0:
                success_count += 1
                print(f"<<< [DONE  {seq_num}/{len(run_list)}] [{datetime.datetime.now().strftime('%H:%M:%S')}] {inst_name} in {elapsed:.1f}s -> UB: {ub_val}, LB: {lb_val}", flush=True)
            else:
                failure_count += 1
                print(f"!!! [FAIL  {seq_num}/{len(run_list)}] [{datetime.datetime.now().strftime('%H:%M:%S')}] {inst_name} (retcode={retcode}) in {elapsed:.1f}s", flush=True)
            update_progress_file()

        return inst_name, retcode, elapsed, ub_val, lb_val

    # Indexed list for clear sequence reporting
    indexed_run_list = [(idx, sz, p) for idx, (sz, p) in enumerate(run_list, 1)]

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = [executor.submit(run_one, item) for item in indexed_run_list]
        for f in concurrent.futures.as_completed(futures):
            if is_shutting_down:
                break

    with lock:
        update_progress_file()

    print("\n" + "=" * 80)
    print("PARALLEL BATCH RUN COMPLETED")
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
