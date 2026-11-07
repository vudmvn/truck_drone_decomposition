#include "bbcp/master/BranchingStrategy.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "bbcp/master/MasterModelBuilder.h"

namespace bbcp
{
    namespace
    {

        bool IsTruckArcAllowed(const Instance &inst, Node i, Node j)
        {
            if (i == j)
                return false;
            if (i == inst.end_depot)
                return false;
            if (j == inst.start_depot)
                return false;
            return true;
        }

        double TruckDirectLB(const Instance &inst, Node i)
        {
            return static_cast<double>(
                inst.truck_time[inst.start_depot][i] +
                inst.truck_time[i][inst.end_depot]);
        }

    } // namespace

    void BranchingStrategySetter::Apply(
        const Instance &inst,
        const MasterOptions &options)
    {
        (void)inst;
        (void)options;

        // This overload is kept only for source compatibility. It cannot set
        // priorities because it does not receive the GRBVar containers.
    }

    void BranchingStrategySetter::Apply(
        const Instance &inst,
        const MasterOptions &options,
        MasterModelData &data,
        double initial_ub)
    {
        (void)initial_ub;

        const int K = inst.n_drones;
        const bool explicit_assignment =
            (options.assignment_mode == MasterAssignmentMode::Explicit);

        // Score components used to rank customer variables. Higher-scored
        // customers receive larger branching priority.
        double max_r = 1.0;
        double max_benefit = 1.0;
        double max_amb_raw = 1.0;

        std::vector<double> r(inst.n + 1, 0.0);
        std::vector<double> energy_ratio(inst.n + 1, 0.0);
        std::vector<double> benefit(inst.n + 1, 0.0);
        std::vector<double> ambiguity(inst.n + 1, 0.0);

        for (Node i = 1; i <= inst.n; ++i)
        {
            r[i] = static_cast<double>(inst.DroneRoundTripTime(i));
            max_r = std::max(max_r, r[i]);

            energy_ratio[i] =
                inst.battery_capacity > 0
                    ? static_cast<double>(inst.energy[i]) /
                          static_cast<double>(inst.battery_capacity)
                    : 1.0;

            const double truck_direct = TruckDirectLB(inst, i);
            benefit[i] = std::max(0.0, truck_direct - r[i]);
            max_benefit = std::max(max_benefit, benefit[i]);

            const double diff = std::abs(truck_direct - r[i]);
            ambiguity[i] = 1.0 / (1.0 + diff);
            max_amb_raw = std::max(max_amb_raw, ambiguity[i]);
        }

        std::vector<double> score(inst.n + 1, 0.0);

        for (Node i = 1; i <= inst.n; ++i)
        {
            const double e_norm = std::min(1.0, std::max(0.0, energy_ratio[i]));
            const double r_norm = r[i] / max_r;
            const double b_norm = benefit[i] / max_benefit;
            const double amb_norm = ambiguity[i] / max_amb_raw;

            double s = 0.0;

            switch (options.branching_strategy)
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

        // ------------------------------------------------------------------
        // Branching priorities.
        // ------------------------------------------------------------------
        // Priority design:
        //   v_k          : highest, fixes active-drone structure early
        //   y_i, t_i     : high, decides truck/drone assignment early
        //   a_{i,k}      : medium-high, assigns customers to specific drones
        //   x_{i,j}      : lower, truck routing arcs are branched after assignment
        //   q_k          : low, auxiliary integer swap lower bound
        if (explicit_assignment)
        {
            for (int k = 0; k < K; ++k)
            {
                data.q[k].set(GRB_IntAttr_BranchPriority, 10);
            }
        }
        else
        {
            data.Q.set(GRB_IntAttr_BranchPriority, 10);
        }

        for (Node i = 1; i <= inst.n; ++i)
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

        const int N = inst.NumNodes();
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (!IsTruckArcAllowed(inst, i, j))
                    continue;
                data.x[i][j].set(GRB_IntAttr_BranchPriority, 50);
            }
        }

        if (!options.use_branching_var_hints)
        {
            return;
        }

        // ------------------------------------------------------------------
        // Optional Gurobi variable hints.
        // ------------------------------------------------------------------
        // These are not hard constraints. They only guide the search toward a
        // plausible incumbent structure.
        std::vector<int> hint_y(inst.n + 1, 0);
        std::vector<Node> drone_candidates;

        for (Node i = 1; i <= inst.n; ++i)
        {
            if (!inst.IsDroneFeasible(i))
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
        for (Node i = 1; i <= inst.n; ++i)
        {
            if (inst.IsDroneFeasible(i))
            {
                feasible_customers.push_back(i);
            }
        }

        std::sort(feasible_customers.begin(), feasible_customers.end(), [&](Node a, Node b)
                  { return score[a] > score[b]; });

        int current_drone_hint_count = 0;
        for (Node i = 1; i <= inst.n; ++i)
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
        std::vector<int> hint_drone(inst.n + 1, -1);
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

        for (Node i = 1; i <= inst.n; ++i)
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

        // Truck arc hints from a greedy nearest-neighbor route over hinted truck
        // customers.
        std::vector<int> used(inst.n + 1, 0);
        Node cur = inst.start_depot;

        for (;;)
        {
            Node best = -1;
            int64_t best_time = std::numeric_limits<int64_t>::max();

            for (Node i = 1; i <= inst.n; ++i)
            {
                if (used[i])
                    continue;
                if (hint_y[i] == 1)
                    continue;

                if (inst.truck_time[cur][i] < best_time)
                {
                    best_time = inst.truck_time[cur][i];
                    best = i;
                }
            }

            if (best == -1)
            {
                break;
            }

            if (IsTruckArcAllowed(inst, cur, best))
            {
                data.x[cur][best].set(GRB_DoubleAttr_VarHintVal, 1.0);
                data.x[cur][best].set(GRB_IntAttr_VarHintPri, 50);
            }

            used[best] = 1;
            cur = best;
        }

        if (IsTruckArcAllowed(inst, cur, inst.end_depot))
        {
            data.x[cur][inst.end_depot].set(GRB_DoubleAttr_VarHintVal, 1.0);
            data.x[cur][inst.end_depot].set(GRB_IntAttr_VarHintPri, 50);
        }
    }

} // namespace bbcp
