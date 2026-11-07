#include "bbcp/master/MasterModelBuilder.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <vector>

#include "bbcp/heuristics/InitialSolution.h"

namespace bbcp
{

    MasterModelBuilder::MasterModelBuilder(
        const Instance &inst,
        const MasterOptions &options) : inst_(inst), options_(options) {}

    bool MasterModelBuilder::IsTruckArcAllowed(Node i, Node j) const
    {
        if (i == j)
            return false;
        if (i == inst_.end_depot)
            return false;
        if (j == inst_.start_depot)
            return false;
        return true;
    }

    double MasterModelBuilder::TruckDirectLB(Node i) const
    {
        return static_cast<double>(
            inst_.truck_time[inst_.start_depot][i] +
            inst_.truck_time[i][inst_.end_depot]);
    }

    double MasterModelBuilder::TruckArcInclusionLB(Node i, Node j) const
    {
        // If arc i -> j is used, the truck route must at least reach i from the
        // start depot, traverse i -> j, and reach the end depot from j.
        double lb = 0.0;

        if (i != inst_.start_depot)
        {
            lb += static_cast<double>(inst_.truck_time[inst_.start_depot][i]);
        }

        lb += static_cast<double>(inst_.truck_time[i][j]);

        if (j != inst_.end_depot)
        {
            lb += static_cast<double>(inst_.truck_time[j][inst_.end_depot]);
        }

        return lb;
    }

    MasterVariablePlan MasterModelBuilder::BuildVariablePlan() const
    {
        MasterVariablePlan plan;

        for (Node i = 1; i <= inst_.n; ++i)
        {
            plan.assignment_truck.push_back("t_" + std::to_string(i));
            plan.assignment_drone_any.push_back("y_" + std::to_string(i));

            for (int k = 0; k < inst_.n_drones; ++k)
            {
                plan.assignment_drone.push_back(
                    "a_" + std::to_string(i) + "_" + std::to_string(k));
            }
        }

        for (int k = 0; k < inst_.n_drones; ++k)
        {
            plan.drone_time_lb.push_back("z_" + std::to_string(k));
        }

        const int N = inst_.NumNodes();

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (!IsTruckArcAllowed(i, j))
                    continue;

                plan.truck_arcs.push_back(
                    "x_" + std::to_string(i) + "_" + std::to_string(j));
            }
        }

        return plan;
    }

    void MasterModelBuilder::AddStaticPreprocessing(
        GRBModel &model,
        MasterModelData &data,
        double initial_ub) const
    {
        const int N = inst_.NumNodes();

        // Fix customers that cannot be drone-served.
        for (Node i = 1; i <= inst_.n; ++i)
        {
            if (!inst_.IsDroneFeasible(i))
            {
                data.y[i].set(GRB_DoubleAttr_LB, 0.0);
                data.y[i].set(GRB_DoubleAttr_UB, 0.0);
                data.t[i].set(GRB_DoubleAttr_LB, 1.0);
                data.t[i].set(GRB_DoubleAttr_UB, 1.0);

                if (options_.assignment_mode == MasterAssignmentMode::Explicit)
                {
                    for (int k = 0; k < inst_.n_drones; ++k)
                    {
                        data.a[i][k].set(GRB_DoubleAttr_LB, 0.0);
                        data.a[i][k].set(GRB_DoubleAttr_UB, 0.0);
                    }
                }
                continue;
            }

            // If a single drone sortie cannot fit under the incumbent UB, then
            // customer i cannot be assigned to any drone in an improving solution.
            if (std::isfinite(initial_ub) &&
                static_cast<double>(inst_.DroneRoundTripTime(i)) > initial_ub + 1e-6)
            {
                data.y[i].set(GRB_DoubleAttr_LB, 0.0);
                data.y[i].set(GRB_DoubleAttr_UB, 0.0);
                data.t[i].set(GRB_DoubleAttr_LB, 1.0);
                data.t[i].set(GRB_DoubleAttr_UB, 1.0);

                if (options_.assignment_mode == MasterAssignmentMode::Explicit)
                {
                    for (int k = 0; k < inst_.n_drones; ++k)
                    {
                        data.a[i][k].set(GRB_DoubleAttr_LB, 0.0);
                        data.a[i][k].set(GRB_DoubleAttr_UB, 0.0);
                    }
                }
            }

            // If even depot -> i -> end is already worse than the incumbent,
            // customer i cannot be on the truck in a strictly improving solution.
            if (std::isfinite(initial_ub) &&
                TruckDirectLB(i) > initial_ub + 1e-6 &&
                inst_.IsDroneFeasible(i))
            {
                data.t[i].set(GRB_DoubleAttr_LB, 0.0);
                data.t[i].set(GRB_DoubleAttr_UB, 0.0);
                data.y[i].set(GRB_DoubleAttr_LB, 1.0);
                data.y[i].set(GRB_DoubleAttr_UB, 1.0);
            }
        }

        // Fix truck arcs that cannot appear in a solution not worse than the
        // incumbent UB.
        if (std::isfinite(initial_ub))
        {
            for (int i = 0; i < N; ++i)
            {
                for (int j = 0; j < N; ++j)
                {
                    if (!IsTruckArcAllowed(i, j))
                        continue;

                    if (TruckArcInclusionLB(i, j) > initial_ub + 1e-6)
                    {
                        data.x[i][j].set(GRB_DoubleAttr_UB, 0.0);
                    }
                }
            }

            data.z.set(GRB_DoubleAttr_UB, initial_ub);
        }

        model.update();
    }

    void MasterModelBuilder::AddStaticStrengtheningCuts(
        GRBModel &model,
        MasterModelData &data,
        double initial_ub) const
    {
        const double ub = std::isfinite(initial_ub) ? initial_ub : 1e12;

        // 1. Truck inclusion lower bound:
        //      z_T >= (c_0i + c_i,end) t_i.
        for (Node i = 1; i <= inst_.n; ++i)
        {
            const double lb = TruckDirectLB(i);
            model.addConstr(
                data.zT >= lb * data.t[i],
                "truck_inclusion_lb_" + std::to_string(i));
        }

        if (options_.assignment_mode != MasterAssignmentMode::Explicit)
        {
            return;
        }

        // 2. Pair drone energy-cover cuts and pair drone-time cuts.
        for (int k = 0; k < inst_.n_drones; ++k)
        {
            for (Node i = 1; i <= inst_.n; ++i)
            {
                if (!inst_.IsDroneFeasible(i))
                    continue;

                for (Node j = i + 1; j <= inst_.n; ++j)
                {
                    if (!inst_.IsDroneFeasible(j))
                        continue;

                    const int64_t eij = inst_.energy[i] + inst_.energy[j];
                    const int needs_swap = (eij > inst_.battery_capacity ? 1 : 0);

                    // If both customers are assigned to drone k and they cannot
                    // fit in one battery, then at least one swap is required.
                    if (needs_swap)
                    {
                        model.addConstr(
                            data.q[k] >= data.a[i][k] + data.a[j][k] - 1.0,
                            "pair_energy_cover_" + std::to_string(i) + "_" +
                                std::to_string(j) + "_" + std::to_string(k));
                    }

                    const double rhs =
                        static_cast<double>(inst_.DroneRoundTripTime(i)) +
                        static_cast<double>(inst_.DroneRoundTripTime(j)) +
                        static_cast<double>(inst_.swap_time) * needs_swap;

                    const double M = std::max(1.0, rhs);

                    model.addConstr(
                        data.zK[k] + M * (2.0 - data.a[i][k] - data.a[j][k]) >= rhs,
                        "pair_drone_time_lb_" + std::to_string(i) + "_" +
                            std::to_string(j) + "_" + std::to_string(k));

                    // 3. No-good from incumbent UB for pairs that cannot be
                    // assigned together to one drone in any improving solution.
                    if (rhs >= ub - 1e-6)
                    {
                        model.addConstr(
                            data.a[i][k] + data.a[j][k] <= 1.0,
                            "pair_drone_ub_nogood_" + std::to_string(i) + "_" +
                                std::to_string(j) + "_" + std::to_string(k));
                    }
                }
            }
        }
    }

    void MasterModelBuilder::ApplyBranchingAndHints(
        MasterModelData &data,
        double initial_ub) const
    {
        (void)initial_ub;

        const int K = inst_.n_drones;
        const bool explicit_assignment =
            (options_.assignment_mode == MasterAssignmentMode::Explicit);

        // Score components.
        double max_r = 1.0;
        double max_benefit = 1.0;
        double max_amb_raw = 1.0;

        std::vector<double> r(inst_.n + 1, 0.0);
        std::vector<double> energy_ratio(inst_.n + 1, 0.0);
        std::vector<double> benefit(inst_.n + 1, 0.0);
        std::vector<double> ambiguity(inst_.n + 1, 0.0);

        for (Node i = 1; i <= inst_.n; ++i)
        {
            r[i] = static_cast<double>(inst_.DroneRoundTripTime(i));
            max_r = std::max(max_r, r[i]);

            energy_ratio[i] =
                inst_.battery_capacity > 0
                    ? static_cast<double>(inst_.energy[i]) /
                          static_cast<double>(inst_.battery_capacity)
                    : 1.0;

            const double truck_direct = TruckDirectLB(i);
            benefit[i] = std::max(0.0, truck_direct - r[i]);
            max_benefit = std::max(max_benefit, benefit[i]);

            const double diff = std::abs(truck_direct - r[i]);
            ambiguity[i] = 1.0 / (1.0 + diff);
            max_amb_raw = std::max(max_amb_raw, ambiguity[i]);
        }

        std::vector<double> score(inst_.n + 1, 0.0);

        for (Node i = 1; i <= inst_.n; ++i)
        {
            const double e_norm = std::min(1.0, std::max(0.0, energy_ratio[i]));
            const double r_norm = r[i] / max_r;
            const double b_norm = benefit[i] / max_benefit;
            const double amb_norm = ambiguity[i] / max_amb_raw;

            double s = 0.0;

            switch (options_.branching_strategy)
            {
            case BranchingStrategy::AssignmentFirst:
                s = 0.50 * b_norm + 0.25 * e_norm + 0.25 * r_norm;
                break;

            case BranchingStrategy::CPAware:
                s = 0.60 * e_norm + 0.40 * r_norm;
                break;

            case BranchingStrategy::AmbiguityFirst:
                s = amb_norm;
                break;

            case BranchingStrategy::DroneBenefit:
                s = b_norm;
                break;

            case BranchingStrategy::Hybrid:
                s = 0.35 * e_norm + 0.25 * r_norm + 0.25 * b_norm + 0.15 * amb_norm;
                break;

            case BranchingStrategy::Default:
            default:
                s = 0.0;
                break;
            }

            score[i] = std::min(1.0, std::max(0.0, s));
        }

        // Variable priorities.
        // No v_k variable is used: all K drones are available by definition.
        // q_k is kept as a lower-bound variable for the number of battery swaps.
        if (explicit_assignment)
        {
            for (int k = 0; k < K; ++k)
            {
                data.q[k].set(GRB_IntAttr_BranchPriority, 20);
            }
        }
        else
        {
            data.Q.set(GRB_IntAttr_BranchPriority, 20);
        }

        for (Node i = 1; i <= inst_.n; ++i)
        {
            const int si = static_cast<int>(100.0 * score[i]);

            data.y[i].set(GRB_IntAttr_BranchPriority, 300 + si);
            data.t[i].set(GRB_IntAttr_BranchPriority, 280 + si);

            if (explicit_assignment)
            {
                for (int k = 0; k < K; ++k)
                {
                    data.a[i][k].set(
                        GRB_IntAttr_BranchPriority,
                        std::max(1, 220 + si - 5 * k));
                }
            }
        }

        const int N = inst_.NumNodes();
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (!IsTruckArcAllowed(i, j))
                    continue;
                data.x[i][j].set(GRB_IntAttr_BranchPriority, 50);
            }
        }

        if (!options_.use_branching_var_hints)
        {
            return;
        }

        // Construct simple hints for y/t/a:
        // 1. Drone if benefit positive and energy not too high.
        // 2. Distribute hinted drone customers over the K anonymous drones.
        std::vector<int> hint_y(inst_.n + 1, 0);
        std::vector<Node> drone_candidates;

        for (Node i = 1; i <= inst_.n; ++i)
        {
            if (!inst_.IsDroneFeasible(i))
            {
                hint_y[i] = 0;
                continue;
            }

            if (benefit[i] > 0.0 && energy_ratio[i] <= 0.85)
            {
                hint_y[i] = 1;
                drone_candidates.push_back(i);
            }
        }

        std::vector<Node> feasible_customers;
        for (Node i = 1; i <= inst_.n; ++i)
        {
            if (inst_.IsDroneFeasible(i))
            {
                feasible_customers.push_back(i);
            }
        }

        std::sort(feasible_customers.begin(), feasible_customers.end(), [&](Node a, Node b)
                  { return score[a] > score[b]; });

        int current_drone_hint_count = 0;
        for (Node i = 1; i <= inst_.n; ++i)
        {
            current_drone_hint_count += hint_y[i];
        }

        for (Node i : feasible_customers)
        {
            if (current_drone_hint_count >= K)
                break;
            if (hint_y[i] == 0)
            {
                hint_y[i] = 1;
                current_drone_hint_count++;
                drone_candidates.push_back(i);
            }
        }

        // Greedy workload balancing for a[i][k] hints in explicit mode only.
        std::vector<int> hint_drone(inst_.n + 1, -1);
        if (explicit_assignment)
        {
            std::vector<double> hinted_workload(K, 0.0);

            std::sort(drone_candidates.begin(), drone_candidates.end(), [&](Node a, Node b)
                      { return score[a] > score[b]; });

            for (Node i : drone_candidates)
            {
                if (K <= 0)
                    break;

                int best_k = 0;
                for (int k = 1; k < K; ++k)
                {
                    if (hinted_workload[k] < hinted_workload[best_k])
                    {
                        best_k = k;
                    }
                }

                hint_drone[i] = best_k;
                hinted_workload[best_k] += r[i];
            }
        }

        for (Node i = 1; i <= inst_.n; ++i)
        {
            data.y[i].set(GRB_DoubleAttr_VarHintVal, static_cast<double>(hint_y[i]));
            data.t[i].set(GRB_DoubleAttr_VarHintVal, 1.0 - static_cast<double>(hint_y[i]));

            data.y[i].set(GRB_IntAttr_VarHintPri, 300);
            data.t[i].set(GRB_IntAttr_VarHintPri, 280);

            if (explicit_assignment)
            {
                for (int k = 0; k < K; ++k)
                {
                    const double val = (hint_drone[i] == k ? 1.0 : 0.0);
                    data.a[i][k].set(GRB_DoubleAttr_VarHintVal, val);
                    data.a[i][k].set(GRB_IntAttr_VarHintPri, 220);
                }
            }
        }

        // Truck arc hints from a greedy/nearest-neighbor route over hint truck customers.
        std::vector<int> used(inst_.n + 1, 0);
        Node cur = inst_.start_depot;

        for (;;)
        {
            Node best = -1;
            int64_t best_time = std::numeric_limits<int64_t>::max();

            for (Node i = 1; i <= inst_.n; ++i)
            {
                if (used[i])
                    continue;
                if (hint_y[i] == 1)
                    continue;

                if (inst_.truck_time[cur][i] < best_time)
                {
                    best_time = inst_.truck_time[cur][i];
                    best = i;
                }
            }

            if (best == -1)
            {
                break;
            }

            if (IsTruckArcAllowed(cur, best))
            {
                data.x[cur][best].set(GRB_DoubleAttr_VarHintVal, 1.0);
                data.x[cur][best].set(GRB_IntAttr_VarHintPri, 50);
            }

            used[best] = 1;
            cur = best;
        }

        if (IsTruckArcAllowed(cur, inst_.end_depot))
        {
            data.x[cur][inst_.end_depot].set(GRB_DoubleAttr_VarHintVal, 1.0);
            data.x[cur][inst_.end_depot].set(GRB_IntAttr_VarHintPri, 50);
        }
    }

    MasterModelData MasterModelBuilder::BuildGurobiModel(
        GRBModel &model,
        double initial_ub) const
    {
        const int N = inst_.NumNodes();

        MasterModelData data;
        const bool explicit_assignment =
            (options_.assignment_mode == MasterAssignmentMode::Explicit);

        data.t.resize(N);
        data.y.resize(N);
        if (explicit_assignment)
        {
            data.a.assign(N, std::vector<GRBVar>(inst_.n_drones));
            data.q.resize(inst_.n_drones);
            data.zK.resize(inst_.n_drones);
        }
        data.x.assign(N, std::vector<GRBVar>(N));
        data.ordT.resize(N);

        // ------------------------------------------------------------
        // Variables.
        // ------------------------------------------------------------
        for (Node i = 1; i <= inst_.n; ++i)
        {
            data.t[i] = model.addVar(
                0.0, 1.0, 0.0, GRB_BINARY,
                "t_" + std::to_string(i));

            data.y[i] = model.addVar(
                0.0, 1.0, 0.0, GRB_BINARY,
                "y_" + std::to_string(i));

            if (explicit_assignment)
            {
                for (int k = 0; k < inst_.n_drones; ++k)
                {
                    data.a[i][k] = model.addVar(
                        0.0, 1.0, 0.0, GRB_BINARY,
                        "a_" + std::to_string(i) + "_" + std::to_string(k));
                }
            }

            if (options_.subtour_mode == MasterSubtourMode::MTZ)
            {
                data.ordT[i] = model.addVar(
                    0.0, static_cast<double>(inst_.n), 0.0, GRB_CONTINUOUS,
                    "ordT_" + std::to_string(i));
            }
        }

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (!IsTruckArcAllowed(i, j))
                    continue;

                data.x[i][j] = model.addVar(
                    0.0, 1.0, 0.0, GRB_BINARY,
                    "x_" + std::to_string(i) + "_" + std::to_string(j));
            }
        }

        if (explicit_assignment)
        {
            for (int k = 0; k < inst_.n_drones; ++k)
            {
                data.q[k] = model.addVar(
                    0.0, static_cast<double>(std::max(0, inst_.n - 1)), 0.0, GRB_INTEGER,
                    "q_" + std::to_string(k));

                data.zK[k] = model.addVar(
                    0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                    "z_" + std::to_string(k));
            }
        }
        else
        {
            data.Q = model.addVar(
                0.0, static_cast<double>(std::max(0, inst_.n - inst_.n_drones)), 0.0,
                GRB_INTEGER, "Q");
        }

        data.zT = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z_T");
        data.zD = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z_D");
        data.z = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "z");

        model.update();

        AddStaticPreprocessing(model, data, initial_ub);

        // ------------------------------------------------------------
        // Assignment.
        //
        // Explicit mode:
        //      t_i + y_i = 1
        //      sum_k a_i_k = y_i
        //
        // Slave-assign mode:
        //      t_i + y_i = 1
        // and the slave receives S={i:y_i=1} and decides which drone serves i.
        // ------------------------------------------------------------
        for (Node i = 1; i <= inst_.n; ++i)
        {
            model.addConstr(
                data.t[i] + data.y[i] == 1.0,
                "truck_or_drone_" + std::to_string(i));

            if (explicit_assignment)
            {
                GRBLinExpr assign_drone = 0.0;
                for (int k = 0; k < inst_.n_drones; ++k)
                {
                    assign_drone += data.a[i][k];

                    model.addConstr(
                        data.a[i][k] <= data.y[i],
                        "force_a_by_y_" + std::to_string(i) + "_" + std::to_string(k));
                }

                model.addConstr(
                    assign_drone == data.y[i],
                    "assign_specific_drone_" + std::to_string(i));
            }
        }

        // ------------------------------------------------------------
        // Truck path constraints.
        // ------------------------------------------------------------
        {
            GRBLinExpr out_start = 0.0;
            for (int j = 0; j < N; ++j)
            {
                if (IsTruckArcAllowed(inst_.start_depot, j))
                {
                    out_start += data.x[inst_.start_depot][j];
                }
            }
            model.addConstr(out_start == 1.0, "start_out");
        }

        {
            GRBLinExpr in_end = 0.0;
            for (int i = 0; i < N; ++i)
            {
                if (IsTruckArcAllowed(i, inst_.end_depot))
                {
                    in_end += data.x[i][inst_.end_depot];
                }
            }
            model.addConstr(in_end == 1.0, "end_in");
        }

        for (Node i = 1; i <= inst_.n; ++i)
        {
            GRBLinExpr out_i = 0.0;
            GRBLinExpr in_i = 0.0;

            for (int j = 0; j < N; ++j)
            {
                if (IsTruckArcAllowed(i, j))
                    out_i += data.x[i][j];
                if (IsTruckArcAllowed(j, i))
                    in_i += data.x[j][i];
            }

            model.addConstr(out_i == data.t[i], "truck_out_" + std::to_string(i));
            model.addConstr(in_i == data.t[i], "truck_in_" + std::to_string(i));
        }

        // Truck subtour handling.
        //
        // MTZ mode: add Miller--Tucker--Zemlin order constraints directly.
        // SEC mode: do not add MTZ variables/constraints; truck subtours are
        // eliminated lazily in MasterCallback at integer solutions.
        if (options_.subtour_mode == MasterSubtourMode::MTZ)
        {
            const double M = static_cast<double>(std::max(1, inst_.n));

            for (Node i = 1; i <= inst_.n; ++i)
            {
                model.addConstr(
                    data.ordT[i] <= M * data.t[i],
                    "ordT_ub_" + std::to_string(i));
                model.addConstr(
                    data.ordT[i] >= data.t[i],
                    "ordT_lb_" + std::to_string(i));
            }

            for (Node i = 1; i <= inst_.n; ++i)
            {
                for (Node j = 1; j <= inst_.n; ++j)
                {
                    if (i == j)
                        continue;
                    if (!IsTruckArcAllowed(i, j))
                        continue;

                    model.addConstr(
                        data.ordT[j] >=
                            data.ordT[i] + 1.0 - M * (1.0 - data.x[i][j]),
                        "mtz_" + std::to_string(i) + "_" + std::to_string(j));
                }
            }
        }
        else
        {
            if (options_.verbose)
            {
                std::cout << "[MASTER] truck subtour mode=sec; "
                          << "MTZ constraints disabled, lazy SEC enabled\n";
            }
        }

        // ------------------------------------------------------------
        // Truck time.
        // ------------------------------------------------------------
        GRBLinExpr truck_cost = 0.0;

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (!IsTruckArcAllowed(i, j))
                    continue;
                truck_cost += static_cast<double>(inst_.truck_time[i][j]) * data.x[i][j];
            }
        }

        model.addConstr(data.zT >= truck_cost, "truck_time_lb");

        // ------------------------------------------------------------
        // Per-drone lower bounds and upper activation bounds.
        // Only used in explicit mode. In slave-assign mode, the master keeps
        // only aggregate drone lower bounds and delegates the K-route assignment
        // to the slave.
        // ------------------------------------------------------------
        if (explicit_assignment)
        {
        for (int k = 0; k < inst_.n_drones; ++k)
        {
            GRBLinExpr assigned_count = 0.0;
            GRBLinExpr workload = 0.0;
            GRBLinExpr energy = 0.0;

            for (Node i = 1; i <= inst_.n; ++i)
            {
                assigned_count += data.a[i][k];
                workload += static_cast<double>(inst_.DroneRoundTripTime(i)) * data.a[i][k];
                energy += static_cast<double>(inst_.energy[i]) * data.a[i][k];

                model.addConstr(
                    data.zK[k] >= static_cast<double>(inst_.DroneRoundTripTime(i)) * data.a[i][k],
                    "single_drone_lb_" + std::to_string(i) + "_" + std::to_string(k));
            }

            // E (1 + q_k) >= sum_i e_i a_i_k.
            model.addConstr(
                static_cast<double>(inst_.battery_capacity) * (1.0 + data.q[k]) >= energy,
                "energy_lb_" + std::to_string(k));

            model.addConstr(data.q[k] <= assigned_count, "q_le_count_" + std::to_string(k));

            // z_k >= sum_i r_i a_i_k + tau q_k.
            model.addConstr(
                data.zK[k] >= workload + static_cast<double>(inst_.swap_time) * data.q[k],
                "drone_workload_lb_" + std::to_string(k));

            model.addConstr(data.zD >= data.zK[k], "zD_ge_zK_" + std::to_string(k));

            if (std::isfinite(initial_ub))
            {
                // Since all K drones are available, zK is simply bounded by the
                // incumbent upper bound. q_k already has a finite variable bound
                // and is additionally bounded by assigned_count above.
                model.addConstr(
                    data.zK[k] <= initial_ub,
                    "zK_le_UB_" + std::to_string(k));
            }
        }

        }

        // Global drone workload valid inequality using y_i:
        //      K z_D >= sum_i r_i y_i + tau sum_k q_k.
        // This is stronger computationally than using sum_k a_{i,k} because it
        // becomes active as soon as the truck/drone assignment y_i is decided.
        if (inst_.n_drones > 0)
        {
            GRBLinExpr total_workload_y = 0.0;
            GRBLinExpr total_energy_y = 0.0;
            GRBLinExpr total_swaps_lb = 0.0;

            if (explicit_assignment)
            {
                for (int k = 0; k < inst_.n_drones; ++k)
                {
                    total_swaps_lb += data.q[k];
                }
            }
            else
            {
                total_swaps_lb += data.Q;
            }

            for (Node i = 1; i <= inst_.n; ++i)
            {
                if (!inst_.IsDroneFeasible(i))
                    continue;

                total_workload_y +=
                    static_cast<double>(inst_.DroneRoundTripTime(i)) * data.y[i];
                total_energy_y +=
                    static_cast<double>(inst_.energy[i]) * data.y[i];
            }

            for (Node i = 1; i <= inst_.n; ++i)
            {
                if (!inst_.IsDroneFeasible(i))
                    continue;
                model.addConstr(
                    data.zD >= static_cast<double>(inst_.DroneRoundTripTime(i)) * data.y[i],
                    "single_customer_drone_lb_y_" + std::to_string(i));
            }

            model.addConstr(
                static_cast<double>(inst_.n_drones) * data.zD >=
                    total_workload_y + static_cast<double>(inst_.swap_time) * total_swaps_lb,
                "global_drone_workload_lb_y");

            // Global minimum number of battery cycles / swaps:
            //      B (K + sum_k q_k) >= sum_i e_i y_i.
            // All K drones are available; each provides one initial battery cycle,
            // and each q_k adds one extra battery cycle.
            if (inst_.battery_capacity > 0)
            {
                model.addConstr(
                    static_cast<double>(inst_.battery_capacity) *
                            (static_cast<double>(inst_.n_drones) + total_swaps_lb) >=
                        total_energy_y,
                    "global_min_swap_energy_lb_y");

                // Cardinality versions of the same idea. If e_i > B/(p+1), then
                // one battery cycle can contain at most p such heavy customers.
                const int max_p = std::min(inst_.n, 5);
                for (int p = 1; p <= max_p; ++p)
                {
                    GRBLinExpr heavy_count = 0.0;
                    const double threshold =
                        static_cast<double>(inst_.battery_capacity) /
                        static_cast<double>(p + 1);

                    for (Node i = 1; i <= inst_.n; ++i)
                    {
                        if (!inst_.IsDroneFeasible(i))
                            continue;
                        if (static_cast<double>(inst_.energy[i]) > threshold + 1e-9)
                        {
                            heavy_count += data.y[i];
                        }
                    }

                    model.addConstr(
                        static_cast<double>(p) *
                                (static_cast<double>(inst_.n_drones) + total_swaps_lb) >=
                            heavy_count,
                        "global_min_swap_cardinality_y_p_" + std::to_string(p));
                }
            }
        }

        if (explicit_assignment)
        {
        // Augmented workload ordering symmetry breaking:
        //      L_0 >= L_1 >= ... >= L_{K-1},
        // where L_k = sum_i r_i a_{i,k} + tau q_k.
        for (int k = 0; k + 1 < inst_.n_drones; ++k)
        {
            GRBLinExpr lk = static_cast<double>(inst_.swap_time) * data.q[k];
            GRBLinExpr lk1 = static_cast<double>(inst_.swap_time) * data.q[k + 1];

            for (Node i = 1; i <= inst_.n; ++i)
            {
                lk += static_cast<double>(inst_.DroneRoundTripTime(i)) * data.a[i][k];
                lk1 += static_cast<double>(inst_.DroneRoundTripTime(i)) * data.a[i][k + 1];
            }

            model.addConstr(lk >= lk1, "sym_augmented_workload_" + std::to_string(k));
        }

        }

        AddStaticStrengtheningCuts(model, data, initial_ub);

        // ------------------------------------------------------------
        // Objective.
        // ------------------------------------------------------------
        model.addConstr(data.z >= data.zT, "z_ge_truck");
        model.addConstr(data.z >= data.zD, "z_ge_drone");

        GRBLinExpr obj = 0.0;
        obj += data.z;
        model.setObjective(obj, GRB_MINIMIZE);

        ApplyBranchingAndHints(data, initial_ub);

        return data;
    }

    void MasterModelBuilder::Build()
    {
        // Exact Gurobi implementation is BuildGurobiModel().
    }

} // namespace bbcp
