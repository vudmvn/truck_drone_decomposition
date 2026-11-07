#!/usr/bin/env python3
"""Apply correctness/CLI updates to vudmvn/truck_drone_decomposition (main, 2026-11-07).

All replacements are guarded by unique anchor assertions. Refuses a partially changed
source tree. Creates .backup files for every changed original source file.
"""
from __future__ import annotations
from pathlib import Path
import argparse
import hashlib
import shutil
import sys

import lipschitz_upgrade as lip


def replace_once(source: str, old: str, new: str, label: str) -> str:
    count = source.count(old)
    if count != 1:
        raise ValueError(f"{label}: expected one anchor; observed {count}. Source likely differs from documented upstream.")
    return source.replace(old, new, 1)


def replace_n(source: str, old: str, new: str, n: int, label: str) -> str:
    count=source.count(old)
    if count != n:
        raise ValueError(f"{label}: expected {n} anchors; observed {count}.")
    return source.replace(old, new)


def patch_cp_header(s: str) -> str:
    s = replace_once(s,
        'explicit SingleDroneOrderingCPSatSlave(const Instance& inst);',
        'explicit SingleDroneOrderingCPSatSlave(const Instance& inst, int requested_drones = 0);',
        'CP ctor declaration')
    s = replace_once(s, '    const Instance& inst_;\n};',
        '    const Instance& inst_;\n    int requested_drones_ = 0; // 1 for explicit mode; 0 means K for slave-assign\n};',
        'CP drone override member')
    return s


def patch_cp_impl(s: str) -> str:
    s = replace_once(s,
        'SingleDroneOrderingCPSatSlave::SingleDroneOrderingCPSatSlave(const Instance& inst)\n    : inst_(inst) {}',
        'SingleDroneOrderingCPSatSlave::SingleDroneOrderingCPSatSlave(const Instance& inst, int requested_drones)\n    : inst_(inst), requested_drones_(requested_drones) {}',
        'CP ctor definition')
    s = replace_once(s,
        '    const int K = inst_.n_drones;',
        '    const int K = requested_drones_ > 0 ? requested_drones_ : inst_.n_drones;\n'
        '    const int H = std::min(K, m); // number of nonempty drone chains',
        'CP effective drones')
    old = '''    // User-requested guard: the K-route slave is used only when there are at
    // least K selected drone customers. In the intended data setting, this is
    // always true. If not, return a zero drone makespan and do not build CP-SAT.
    if (m == 0 || m < K || K <= 0) {'''
    new = '''    // Empty work set is the ONLY case with zero drone makespan.
    // For 0 < m < K, use H=m nonempty chains and K-H idle drones.
    if (m == 0) {'''
    s=replace_once(s,old,new,'CP invalid m<K guard')
    s=replace_once(s,
        '    if (inst_.battery_capacity <= 0) {',
        '    if (K <= 0 || inst_.battery_capacity <= 0) {',
        'CP invalid K guard')
    s=replace_once(s,'    const int64_t max_swaps = std::max(0, m - K);',
        '    const int64_t max_swaps = std::max(0, m - H);',
        'CP number possible swaps')
    s=replace_once(s,
        '    // Exactly K non-empty routes.',
        '    // Exactly H=min(K,m) non-empty routes; K-H drones may remain idle.',
        'CP comment paths')
    s=replace_once(s,
        '        cp.AddEquality(sum_heads, K);\n        cp.AddEquality(sum_tails, K);',
        '        cp.AddEquality(sum_heads, H);\n        cp.AddEquality(sum_tails, H);',
        'CP chain count')
    s=replace_once(s,
        '        cp.AddEquality(internal_arcs, m - K);',
        '        cp.AddEquality(internal_arcs, m - H);',
        'CP internal arcs')
    s=replace_once(s,
        '''            if (i > j) {
                cp.AddEquality(swap_before[q], 1).OnlyEnforceIf(u[p][q]);
            }
''',
        '''            if (i > j) {
                cp.AddEquality(swap_before[q], 1).OnlyEnforceIf(u[p][q]);
            }
            // Redundant, but propagates battery incompatibility early.
            if (inst_.energy[i] + inst_.energy[j] > inst_.battery_capacity) {
                cp.AddEquality(swap_before[q], 1).OnlyEnforceIf(u[p][q]);
            }
''','CP battery incompatibility')
    s=replace_once(s,
        '''    if (response.status() == CpSolverStatus::INFEASIBLE) {
        res.proven_infeasible = true;
        return res;
    }

    if (response.status() != CpSolverStatus::OPTIMAL &&
        response.status() != CpSolverStatus::FEASIBLE) {
        res.unknown = true;
        return res;
    }
''',
        '''    // Preserve certified solver bounds even on a time-limited UNKNOWN.
    // We retain the analytic LB and only improve it using the CP-SAT dual bound.
    if (response.status() != CpSolverStatus::MODEL_INVALID) {
        const double dual = response.best_objective_bound();
        if (std::isfinite(dual) && dual >= 0.0 &&
            dual <= static_cast<double>(std::numeric_limits<int64_t>::max() / 8)) {
            const int64_t bound = static_cast<int64_t>(std::ceil(dual - 1e-7));
            res.lower_bound = std::max(res.lower_bound, bound);
            res.lower_bound_valid = true;
        }
    }

    if (response.status() == CpSolverStatus::INFEASIBLE) {
        res.proven_infeasible = true;
        // Infeasibility under a deadline only certifies phi(D) >= deadline+1.
        if (options.makespan_cutoff >= 0) {
            res.lower_bound = std::max(res.lower_bound, options.makespan_cutoff + 1);
            res.lower_bound_valid = true;
        }
        return res;
    }

    if (response.status() != CpSolverStatus::OPTIMAL &&
        response.status() != CpSolverStatus::FEASIBLE) {
        res.unknown = true;
        return res;
    }
''','CP status / bounds')
    s=replace_once(s,
        '    return "k-drone-ordering-cpsat";',
        '    return requested_drones_ == 1 ? "one-drone-ordering-cpsat" : "k-drone-ordering-cpsat";',
        'CP backend display name')
    # Required by new code's checked casts.
    s=replace_once(s,'#include <iostream>\n', '#include <iostream>\n#include <limits>\n', 'CP includes')
    s=replace_once(s,
        '''    } else {
        res.lower_bound = static_cast<int64_t>(std::ceil(response.best_objective_bound()));
        res.lower_bound_valid = true;
    }
''',
        '''    } else {
        // Dual bound already extracted above; preserve the strongest LB.
        res.lower_bound_valid = true;
    }
''','CP feasible nonoptimal LB')
    # For a single drone and zero swap cost, exact objective is sum of flight times.
    # Handles CP timeout for a particularly common instance family.
    marker = '''    const int64_t min_cycles = SafeCeilDiv(total_energy, inst_.battery_capacity);'''
    addition='''    if (K == 1 && inst_.swap_time == 0) {
        if (options.makespan_cutoff >= 0 && total_work > options.makespan_cutoff) {
            res.proven_infeasible = true;
            res.lower_bound = total_work;
            res.lower_bound_valid = true;
            return res;
        }
        res.feasible = true;
        res.optimal = true;
        res.objective = total_work;
        res.lower_bound = total_work;
        res.lower_bound_valid = true;
        res.chains.assign(1, drone_customers);
        res.swap_before.assign(1, std::vector<int>(m, 1));
        if (!res.swap_before.front().empty()) res.swap_before.front().front() = 0;
        return res;
    }

'''
    s=replace_once(s,marker,addition+marker,'CP zero swap singleton')
    return s


def patch_factory_header(s: str) -> str:
    return replace_once(s,
        'CreateSlaveSolver(const Instance& inst, SlaveSolverType type);',
        'CreateSlaveSolver(const Instance& inst, SlaveSolverType type, MasterAssignmentMode assignment_mode);',
        'factory declaration')


def patch_factory_impl(s: str) -> str:
    start=s.index('std::unique_ptr<ISlaveSolver>\nCreateSlaveSolver(')
    ending=s.index('\n}  // namespace bbcp',start)
    s=s[:start]+'''std::unique_ptr<ISlaveSolver>
CreateSlaveSolver(const Instance& inst, SlaveSolverType type,
                  MasterAssignmentMode assignment_mode) {
    if (inst.n_drones <= 0) {
        throw std::invalid_argument("Number of drones must be >= 1");
    }

    if (assignment_mode == MasterAssignmentMode::SlaveAssign) {
        if (type == SlaveSolverType::MILP) {
            throw std::invalid_argument(
                "Unsupported combination: slave-assign + MILP. "
                "The MILP implementation orders ONE drone, not K drones. "
                "Choose --assignment slave-assign --slave cp instead.");
        }
#ifdef BBCP_WITH_ORTOOLS
        return std::make_unique<SingleDroneOrderingCPSatSlave>(inst, 0);
#else
        throw std::runtime_error(
            "slave-assign requires OR-Tools K-drone CP-SAT. "
            "Rebuild with -DBBCP_WITH_ORTOOLS=ON or choose explicit+milp.");
#endif
    }

    if (type == SlaveSolverType::MILP) {
        return std::make_unique<SingleDroneOrderingMILPSlave>(inst);
    }
#ifdef BBCP_WITH_ORTOOLS
    // Explicit master fixes S_k. Each call must use exactly ONE drone.
    return std::make_unique<SingleDroneOrderingCPSatSlave>(inst, 1);
#else
    throw std::runtime_error(
        "explicit+cp requires OR-Tools. Choose --slave milp or rebuild "
        "with -DBBCP_WITH_ORTOOLS=ON. No silent fallback is performed.");
#endif
}
'''+s[ending:]
    return s


def patch_builder(s: str) -> str:
    # MTZ n+1, otherwise unselected customer ord=0 can invalidate feasible paths.
    s=replace_once(s,
        'const double M = static_cast<double>(std::max(1, inst_.n));',
        'const double M = static_cast<double>(std::max(1, inst_.n + 1));',
        'correct MTZ big-M')
    s=replace_once(s,
        'if (rhs >= ub - 1e-6)',
        'if (rhs > ub + 1e-6)',
        'pair preprocessing tied to valid INITIAL upper bound')
    return s


def patch_callback(s: str) -> str:
    # Check every solver-produced drone schedule before treating it as a valid UB.
    checker = r"""namespace {

bool ValidateDecodedSlaveSchedule(const Instance& inst,
                                  const std::vector<Node>& expected,
                                  const SlaveResult& result,
                                  int max_chains) {
    if (!result.feasible || result.objective < 0 ||
        static_cast<int>(result.chains.size()) > max_chains ||
        result.swap_before.size() != result.chains.size()) return false;

    std::vector<Node> actual;
    int64_t makespan = 0;
    for (size_t k=0; k<result.chains.size(); ++k) {
        const auto& chain = result.chains[k];
        const auto& swaps = result.swap_before[k];
        if (swaps.size() != chain.size()) return false;
        int64_t battery = inst.battery_capacity;
        int64_t time = 0;
        for (size_t p=0; p<chain.size(); ++p) {
            const Node i = chain[p];
            if (!inst.IsCustomer(i) || !inst.IsDroneFeasible(i)) return false;
            if (swaps[p] != 0 && swaps[p] != 1) return false;
            if (p == 0 && swaps[p] != 0) return false;
            if (swaps[p] == 1) {
                battery = inst.battery_capacity;
                time += inst.swap_time;
            }
            if (battery < inst.energy[i]) return false;
            battery -= inst.energy[i];
            time += inst.DroneRoundTripTime(i);
            actual.push_back(i);
        }
        makespan = std::max(makespan,time);
    }
    std::vector<Node> sorted_expected = expected;
    std::sort(sorted_expected.begin(),sorted_expected.end());
    std::sort(actual.begin(),actual.end());
    return actual == sorted_expected &&
           static_cast<double>(makespan) <= static_cast<double>(result.objective)+1e-6;
}

} // namespace
"""
    s=replace_once(s,
        'namespace bbcp\n{\n',
        'namespace bbcp\n{\n\n'+checker+'\n',
        'insert decoded schedule validator')
    # Never turn infeasibility of a deadline-restricted slave into a global no-good.
    s=replace_once(s,
        '''                    // The set S cannot be served by one drone under UB-1.
                    // Since drones are identical, forbid this set on every drone.
                    AddBroadcastDroneNoGoodCut(S);
                    AddBroadcastDroneSubsetLowerBoundCut(S, best_valid_ub_);''',
        '''                    // Deadline INFEASIBLE is NOT global infeasibility.
                    // A certified integer deadline T implies phi(S)>=T+1.
                    if (slave_opts.makespan_cutoff >= 0) {
                        AddBroadcastDroneSubsetLowerBoundCut(
                            S, static_cast<double>(slave_opts.makespan_cutoff + 1));
                    } else {
                        std::cerr << "[ERROR] Unrestricted slave INFEASIBLE; "
                                     "cannot safely generate a cutoff no-good.\\n";
                        abort();
                        return;
                    }''','explicit cutoff proof')
    s=replace_once(s,
        '''                else if (r.lower_bound_valid)
                {
                    AddBroadcastDroneSubsetLowerBoundCut(S, static_cast<double>(r.lower_bound));
                }
                else
                {
                    unresolved_invalid_candidate = true;''',
        '''                else if (r.lower_bound_valid &&
                         static_cast<double>(r.lower_bound) >
                             getSolution(data_.zK[k]) + 1e-6)
                {
                    AddBroadcastDroneSubsetLowerBoundCut(S, static_cast<double>(r.lower_bound));
                }
                else
                {
                    unresolved_invalid_candidate = true;''','explicit unknown must violate')
    s=replace_once(s,
        '            drone_values[k] = static_cast<double>(r.objective);',
        '            if (!ValidateDecodedSlaveSchedule(inst_, S, r, 1)) {\n'
        '                std::cerr << "[ERROR] Invalid one-drone slave schedule: "\n'
        '                             "missing/duplicate customer, swap, or duration.\\n";\n'
        '                abort();\n'
        '                return;\n'
        '            }\n'
        '            drone_values[k] = static_cast<double>(r.objective);',
        'validate explicit decoded CP/MILP result')
    # On a feasible but non-optimal slave, an LB cut may not be violated.
    # If the CP schedule exceeds master zK and LB cannot cut, exact certification is incomplete.
    marker='''            const double zK_master = getSolution(data_.zK[k]);

            if (r.optimal)'''
    new='''            const double zK_master = getSolution(data_.zK[k]);
            if (static_cast<double>(r.objective) > zK_master + 1e-6 &&
                !r.optimal &&
                (!r.lower_bound_valid ||
                 static_cast<double>(r.lower_bound) <= zK_master + 1e-6)) {
                unresolved_invalid_candidate = true;
                unresolved_drone = k;
                unresolved_slave_obj = static_cast<double>(r.objective);
                unresolved_slave_lb = r.lower_bound_valid ? r.lower_bound : -1;
                unresolved_master_zk = zK_master;
            }

            if (r.optimal)'''
    s=replace_once(s,marker,new,'explicit feasible unproven')

    # Eliminate mistaken 'm<K=>0' branch: always call exact K-drone slave.
    old='''        if (static_cast<int>(drone_customers.size()) < K)
        {
            // User-requested guard for the K-route slave: if fewer than K drone
            // customers are selected, do not build the slave. Return a zero
            // drone makespan. In the intended benchmark setting, |S| >= K.
            r.feasible = true;
            r.optimal = true;
            r.objective = 0;
            r.lower_bound = 0;
            r.lower_bound_valid = true;

            if (options_.verbose)
            {
                std::cout << "  [SLAVE-SKIP] |S| < K; return drone objective 0"
                          << ", |S|=" << drone_customers.size()
                          << ", K=" << K << "\\n";
            }
        }
        else
        {'''
    new='''        // Always solve the K-drone slave, including 0 < |D| < K.
        // The slave uses min(K,|D|) nonempty chains; idle drones are allowed.
        {'''
    s=replace_once(s,old,new,'slave-assign invalid zero makespan')
    s=replace_once(s,
        '        SlaveResult r;\n\n        // Always solve the K-drone slave',
        '        SlaveResult r;\n        int64_t active_slave_cutoff = -1;\n\n        // Always solve the K-drone slave',
        'track current K-drone deadline')
    s=replace_once(s,
        '            PrintSlaveCall(-1, drone_customers, slave_opts.makespan_cutoff);',
        '            active_slave_cutoff = slave_opts.makespan_cutoff;\n'
        '            PrintSlaveCall(-1, drone_customers, slave_opts.makespan_cutoff);',
        'remember effective K-drone deadline')
    s=replace_once(s,
        '''            if (r.proven_infeasible)
            {
                AddGlobalDroneNoGoodCut(drone_customers);
            }
            else if (r.lower_bound_valid)
            {
                AddGlobalDroneSubsetLowerBoundCut(drone_customers, static_cast<double>(r.lower_bound));
            }
            else
            {
                std::cerr << "\\n[ERROR] slave-assign slave returned UNKNOWN without a valid bound.\\n"
                          << "        Increase slave_time or use an exact slave.\\n";
                abort();
            }
            return;''',
        '''            if (r.proven_infeasible && active_slave_cutoff < 0) {
                std::cerr << "[ERROR] Slave globally infeasible; no model-safe "
                             "cut generated.\\n";
                abort();
                return;
            }
            const double certified_lb = r.proven_infeasible &&
                    active_slave_cutoff >= 0
                ? static_cast<double>(active_slave_cutoff + 1)
                : (r.lower_bound_valid ? static_cast<double>(r.lower_bound) : -1.0);
            if (certified_lb > zD_master + 1e-6) {
                AddGlobalDroneSubsetLowerBoundCut(drone_customers, certified_lb);
            } else {
                std::cerr << "[ERROR] Unresolved K-drone subproblem: "
                             "no violated certified cut, cannot accept master "
                             "incumbent.\\n";
                abort();
            }
            return;''','slave-assign no-good replace')
    # If a feasible non-optimal CP schedule exceeds master zD but its dual bound
    # cannot separate, stop rather than trusting an unverified integer candidate.
    marker='''        schedule.makespan = static_cast<double>(r.objective);

        const int chains_to_copy'''
    new='''        if (!r.optimal && static_cast<double>(r.objective) > zD_master + 1e-6 &&
            (!r.lower_bound_valid ||
             static_cast<double>(r.lower_bound) <= zD_master + 1e-6)) {
            std::cerr << "[ERROR] Feasible but unresolved K-drone proof; "
                         "increase --slave-time.\\n";
            abort();
            return;
        }
        schedule.makespan = static_cast<double>(r.objective);

        const int chains_to_copy'''
    s=replace_once(s,marker,new,'slave-assign nonoptimal proof')
    s=replace_once(s,
        '        schedule.makespan = static_cast<double>(r.objective);',
        '        if (!ValidateDecodedSlaveSchedule(inst_, drone_customers, r, K)) {\n'
        '            std::cerr << "[ERROR] Invalid K-drone slave schedule.\\n";\n'
        '            abort();\n'
        '            return;\n'
        '        }\n'
        '        schedule.makespan = static_cast<double>(r.objective);',
        'validate slave-assign decoded CP result')
    # Ensure we never keep optimizing after callback failures unnoticed.
    s=replace_once(s,
        '''            std::cerr << "[CALLBACK-GUROBI-ERROR] " << e.getMessage() << "\\n";''',
        '''            std::cerr << "[CALLBACK-GUROBI-ERROR] " << e.getMessage() << "\\n";
            abort();''', 'callback gurobi exceptions')
    s=replace_once(s,
        '''            std::cerr << "[CALLBACK-ERROR] " << e.what() << "\\n";''',
        '''            std::cerr << "[CALLBACK-ERROR] " << e.what() << "\\n";
            abort();''', 'callback C++ exceptions')
    # In return from INFEASIBLE, deadline's cutoff may have been tightened.
    # We replace UB-1 with the largest INTEGER makespan strictly below UB.
    s=replace_n(s,
        'static_cast<int64_t>(std::floor(best_valid_ub_ - 1.0 + 1e-9))',
        'static_cast<int64_t>(std::ceil(best_valid_ub_ - 1e-9) - 1)',
        2,'integer cutoff from real-valued valid UB')
    return s


def patch_master_problem(s: str) -> str:
    s=replace_once(s,
        '#include <iostream>\n',
        '#include <algorithm>\n#include <iostream>\n',
        'MasterProblem algorithm header')
    s=replace_once(s,
        'CreateSlaveSolver(inst_, options_.slave_solver);',
        'CreateSlaveSolver(inst_, options_.slave_solver, options_.assignment_mode);',
        'backend dispatch by assignment mode')
    s=replace_once(s,
        '''if (best.feasible && options_.local_search_options.enabled)''',
        '''if (best.feasible && options_.local_search_options.enabled &&
                options_.assignment_mode == MasterAssignmentMode::Explicit)''',
        'LS explicit-only')
    marker='''            FullSolution best = cb.BestSolution();
'''
    new='''            FullSolution best = cb.BestSolution();
            best.stats.gurobi_status = model.get(GRB_IntAttr_Status);
            best.stats.runtime_sec = model.get(GRB_DoubleAttr_Runtime);
            best.stats.best_valid_ub = cb.BestValidUB();
            best.stats.initial_valid_ub = cb.InitialValidUB();
            best.stats.best_master_lb_seen = cb.BestMasterLBSeen();
            best.stats.time_to_best_solution_sec = cb.TimeToBestSolutionSec();
            best.stats.time_to_1pct_gap_sec = cb.TimeToOnePercentGapSec();
            best.stats.first_1pct_gap_value = cb.FirstOnePercentGapValue();
            best.stats.first_1pct_gap_bound = cb.FirstOnePercentGapBound();
            best.stats.valid_incumbent_updates = cb.ValidIncumbentUpdates();
            try { best.stats.lower_bound = model.get(GRB_DoubleAttr_ObjBound); }
            catch (...) { best.stats.lower_bound = -1.0; }
            if (model.get(GRB_IntAttr_SolCount) > 0) {
                best.stats.master_obj = model.get(GRB_DoubleAttr_ObjVal);
                try { best.stats.mip_gap = model.get(GRB_DoubleAttr_MIPGap); }
                catch (...) { best.stats.mip_gap = -1.0; }
            }
            if (best.stats.gurobi_status != GRB_OPTIMAL) {
                std::cout << "[STATUS] Master stopped without global proof: "
                          << best.stats.gurobi_status << "\\n";
            }
'''
    s=replace_once(s,marker,new,'master statistics')
    s=replace_once(s,
        '            ResultWriter::WriteCSV(options_.result_file, inst_, options_, best);',
        '            if (best.feasible) {\n'
        '                best.stats.best_valid_ub = std::min(best.stats.best_valid_ub, best.objective);\n'
        '            }\n'
        '            ResultWriter::WriteCSV(options_.result_file, inst_, options_, best);',
        'post-LocalSearch UB metadata')
    return s


def patch_options(s: str) -> str:
    s=replace_once(s,
        'bool enable_dynamic_ub_workload_usercut = true;',
        'bool enable_dynamic_ub_workload_usercut = false; // unsafe without a global cutoff proof',
        'disable dynamic UB workload')
    s=replace_once(s,
        'bool enable_dynamic_ub_energy_workload_usercut = true;',
        'bool enable_dynamic_ub_energy_workload_usercut = false; // same',
        'disable UB energy')
    s=replace_once(s,
        'bool enable_slave_assign_ub_incompatibility_usercuts = true;',
        'bool enable_slave_assign_ub_incompatibility_usercuts = false; // UB no-good not globally valid',
        'disable UB no-good')
    return s


def main():
    pa=argparse.ArgumentParser(description=__doc__)
    pa.add_argument('repo',type=Path,help='Path to checked-out truck_drone_decomposition')
    pa.add_argument('--check',action='store_true',help='Verify all anchored changes without writing')
    pa.add_argument('--output-dir',type=Path,default=None,
                    help='Export ALL complete updated .cpp/.h files to a separate overlay, leaving source tree unchanged')
    args=pa.parse_args()
    root=args.repo.resolve()
    package=Path(__file__).resolve().parent.parent
    transforms={
        'include/bbcp/slave/SingleDroneOrderingCPSatSlave.h':patch_cp_header,
        'src/slave/SingleDroneOrderingCPSatSlave.cpp':patch_cp_impl,
        'include/bbcp/slave/SlaveSolverFactory.h':patch_factory_header,
        'src/master/MasterModelBuilder.cpp':lambda s: lip.patch_builder(patch_builder(s)),
        'src/master/MasterCallback.cpp':lambda s: lip.patch_callback(patch_callback(s)),
        'src/master/MasterProblem.cpp':patch_master_problem,
        'include/bbcp/core/Options.h':patch_options,
        'include/bbcp/master/MasterCuts.h':lip.patch_master_cuts_header,
        'src/master/MasterCuts.cpp':lip.patch_master_cuts_impl,
    }
    rewritten={}
    for rel,fn in transforms.items():
        file=root/rel
        if not file.is_file():
            raise SystemExit(f'Missing source file: {file}')
        original=file.read_text(encoding='utf-8')
        rewritten[rel]=fn(original)
        print('PATCH',rel,'changed:',len(original),'->',len(rewritten[rel]))

    # Two smaller source files are full replacements, not patch hunks.
    replacements={
        'src/main.cpp':package/'src/main.cpp',
        'src/slave/SlaveSolverFactory.cpp':package/'src/SlaveSolverFactory.cpp',
    }
    for rel,path in replacements.items():
        if not (root/rel).is_file(): raise SystemExit(f'Missing original: {rel}')
        rewritten[rel]=path.read_text(encoding='utf-8')
        print('REPLACE',rel,'bytes=',len(rewritten[rel]))
    # New header: created, not patched or overwritten. Compile included from the
    # two modified C++ translation units.
    new_header = 'include/bbcp/master/LipschitzCuts.h'
    if (root/new_header).exists():
        raise SystemExit(f'{new_header} already exists; do not apply twice')
    rewritten[new_header]=(package/new_header).read_text(encoding='utf-8')
    print('CREATE', new_header)
    if args.check:
        print('All replacement anchors verified. No files written.')
        return
    if args.output_dir is not None:
        overlay=args.output_dir.resolve()
        for rel,updated in rewritten.items():
            dst=overlay/rel
            if dst.exists():
                raise SystemExit(f'Overlay destination already exists: {dst}')
        for rel,updated in rewritten.items():
            dst=overlay/rel
            dst.parent.mkdir(parents=True,exist_ok=True)
            dst.write_text(updated,encoding='utf-8')
        print('Exported',len(rewritten),'COMPLETE modified source files to',overlay)
        print('Original repository remains unchanged. Copy overlay src/ and include/ files into the repository when ready.')
        return
    # All preconditions must pass before the first file is modified.
    for rel in rewritten:
        dest=root/rel
        if Path(str(dest)+'.backup').exists():
            raise SystemExit(f'Existing backup for {rel}; refusing partial update.')
    for rel,updated in rewritten.items():
        dest=root/rel
        backup=Path(str(dest)+'.backup')
        if dest.exists():
            shutil.copy2(dest,backup)
        else:
            dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_text(updated,encoding='utf-8')
    print('\nUPDATED',len(rewritten),'files. Backups: *.backup. No GitHub push performed.')
    print('Run: cmake -S . -B build -DBBCP_WITH_GUROBI=ON -DBBCP_WITH_ORTOOLS=ON')
    print('Then: ./build/truck_drone_solver --list-variants')

if __name__=='__main__':
    main()
