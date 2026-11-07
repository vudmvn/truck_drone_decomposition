#include "bbcp/master/MasterProblem.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "bbcp/heuristics/InitialSolution.h"
#include "bbcp/heuristics/LocalSearch.h"
#include "bbcp/io/ResultWriter.h"
#include "bbcp/master/MasterCallback.h"
#include "bbcp/master/MasterModelBuilder.h"
#include "bbcp/slave/SlaveSolverFactory.h"

#include "gurobi_c++.h"

namespace bbcp
{

    MasterProblem::MasterProblem(const Instance &inst, const MasterOptions &options)
        : inst_(inst), options_(options) {}

    FullSolution MasterProblem::Solve()
    {
        if (options_.verbose)
        {
            std::cout << "[SOLVE] exact decomposition master"
                      << ", instance=" << inst_.name
                      << ", n=" << inst_.n
                      << ", drones=" << inst_.n_drones
                      << ", slave=" << SlaveSolverTypeName(options_.slave_solver)
                      << "\n";
        }

        // This is only a valid initial incumbent / upper bound.
        // It is not a fallback algorithm and is not returned unless the exact
        // decomposition master cannot improve it.
        FullSolution incumbent = InitialSolution::GreedyAllTruck(inst_);

        if (options_.verbose)
        {
            std::cout << "[INIT] all-truck valid UB=" << incumbent.objective << "\n";
        }

        try
        {
            GRBEnv env(true);
            env.set(GRB_IntParam_OutputFlag, options_.verbose ? 1 : 0);
            env.start();

            GRBModel model(env);
            model.set(GRB_StringAttr_ModelName, "truck_drone_decomposition_master");

            model.set(GRB_IntParam_LazyConstraints, 1);
            model.set(GRB_IntParam_PreCrush, 1);

            // The master relaxation is the bottleneck for proving lower bounds.
            // These settings prioritize bound improvement and allow more aggressive
            // generic cutting planes in addition to our MIPNODE user cuts.
            model.set(GRB_IntParam_MIPFocus, 3);
            model.set(GRB_IntParam_Cuts, 2);

            if (options_.time_limit_seconds > 0.0)
            {
                model.set(GRB_DoubleParam_TimeLimit, options_.time_limit_seconds);
            }

            if (options_.mip_gap > 0.0)
            {
                model.set(GRB_DoubleParam_MIPGap, options_.mip_gap);
            }

            MasterModelBuilder builder(inst_, options_);
            MasterModelData data = builder.BuildGurobiModel(model, incumbent.objective);

            // The decomposition always uses a single-drone slave because the master
            // explicitly assigns each drone-served customer to a drone k.
            // Default option: CP-SAT. Alternative: MILP.
            auto slave = CreateSlaveSolver(inst_, options_.slave_solver, options_.assignment_mode);

            if (options_.verbose)
            {
                std::cout << "[SLAVE] requested=" << SlaveSolverTypeName(options_.slave_solver)
                          << ", actual=" << slave->Name() << "\n";
            }

            MasterCallback cb(inst_, options_, data, *slave, incumbent);
            model.setCallback(&cb);

            model.optimize();

            FullSolution best = cb.BestSolution();
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
                          << best.stats.gurobi_status << "\n";
            }

            if (options_.verbose)
            {
                std::cout << "\n=== Exact decomposition finished ===\n";
                std::cout << "Best valid UB: " << cb.BestValidUB() << "\n";
            }

            // Optional improvement of a solver-verified incumbent only.
            // This is not used as a fallback replacement for the master.
            if (best.feasible && options_.local_search_options.enabled &&
                options_.assignment_mode == MasterAssignmentMode::Explicit)
            {
                LocalSearch ls(inst_, options_.local_search_options, *slave);
                FullSolution improved;
                if (ls.Improve(best, improved) && improved.objective < best.objective)
                {
                    best = improved;
                    if (options_.verbose)
                    {
                        std::cout << "[LS NEW BEST] objective=" << best.objective << "\n";
                    }
                }
            }

            if (best.feasible) {
                best.stats.best_valid_ub = std::min(best.stats.best_valid_ub, best.objective);
            }
            ResultWriter::WriteCSV(options_.result_file, inst_, options_, best);
            ResultWriter::WriteSolutionText("results/solutions/solution.txt", inst_, best);
            if (!options_.sqlite_db_path.empty()) {
                ResultWriter::WriteSQLite(options_.sqlite_db_path, inst_, options_, best);
            }

            return best;
        }
        catch (const GRBException &e)
        {
            std::cerr << "[GUROBI ERROR] " << e.getMessage() << "\n";
            throw;
        }
    }

} // namespace bbcp


