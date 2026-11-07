#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"
#include "bbcp/core/Solution.h"
#include "bbcp/master/MasterModelBuilder.h"
#include "bbcp/slave/ISlaveSolver.h"

#include "gurobi_c++.h"

namespace bbcp
{

    class MasterCallback : public GRBCallback
    {
    public:
        MasterCallback(
            const Instance &inst,
            const MasterOptions &options,
            const MasterModelData &data,
            const ISlaveSolver &slave,
            const FullSolution &initial_incumbent);

        const FullSolution &BestSolution() const;

        double InitialValidUB() const;
        double BestValidUB() const;
        double BestMasterLBSeen() const;

        double TimeToBestSolutionSec() const;
        double TimeToOnePercentGapSec() const;

        double FirstOnePercentGapValue() const;
        double FirstOnePercentGapBound() const;

        int ValidIncumbentUpdates() const;

    protected:
        void callback() override;

    private:
        const Instance &inst_;
        MasterOptions options_;
        const MasterModelData &data_;
        const ISlaveSolver &slave_;

        FullSolution best_solution_;
        double best_valid_ub_ = GRB_INFINITY;

        double last_dynamic_ub_workload_cut_ub_ = GRB_INFINITY;
        double pending_dynamic_ub_workload_cut_ub_ = GRB_INFINITY;
        bool has_pending_dynamic_ub_workload_cut_ = false;

        double last_dynamic_ub_energy_workload_cut_ub_ = GRB_INFINITY;
        double pending_dynamic_ub_energy_workload_cut_ub_ = GRB_INFINITY;
        bool has_pending_dynamic_ub_energy_workload_cut_ = false;

        double initial_valid_ub_ = GRB_INFINITY;
        double best_master_lb_seen_ = -GRB_INFINITY;

        double time_to_best_solution_sec_ = -1.0;

        double time_to_1pct_gap_sec_ = -1.0;
        double first_1pct_gap_value_ = -1.0;
        double first_1pct_gap_bound_ = -1.0;

        int valid_incumbent_updates_ = 0;

        void HandleIntegerSolution();
        void HandleIntegerSolutionExplicit();
        void HandleIntegerSolutionSlaveAssign();

        bool SeparateIntegerTruckSubtourCuts(
            const std::vector<int> &tbar,
            const std::vector<std::vector<int>> &xbar);

        std::vector<std::vector<Node>> FindTruckSubtours(
            const std::vector<int> &tbar,
            const std::vector<std::vector<int>> &xbar) const;

        void AddTruckSubtourEliminationCut(
            const std::vector<Node> &S);

        std::vector<Node> DecodeTruckRoute(
            const std::vector<std::vector<int>> &xbar) const;

        double ComputeTruckValue(
            const std::vector<std::vector<int>> &xbar) const;

        void AddDroneSubsetLowerBoundCut(
            int drone_id,
            const std::vector<Node> &S,
            double rhs);

        void AddBroadcastDroneSubsetLowerBoundCut(
            const std::vector<Node> &S,
            double rhs);

        void AddGlobalDroneSubsetLowerBoundCut(
            const std::vector<Node> &S,
            double rhs);

        void AddDroneNoGoodCut(
            int drone_id,
            const std::vector<Node> &S);

        void AddBroadcastDroneNoGoodCut(
            const std::vector<Node> &S);

        void AddGlobalDroneNoGoodCut(
            const std::vector<Node> &S);

        std::string NodeListString(const std::vector<Node> &nodes) const;

        void PrintSlaveCall(
            int drone_id,
            const std::vector<Node> &S,
            int64_t cutoff) const;

        void PrintSlaveResult(
            int drone_id,
            const std::vector<Node> &S,
            int64_t cutoff,
            const SlaveResult &result) const;

        void SubmitRepairedIncumbent(
            const std::vector<int> &tbar,
            const std::vector<std::vector<int>> &abar,
            const std::vector<std::vector<int>> &xbar,
            const std::vector<double> &drone_values,
            const std::vector<int> &swap_counts,
            double truck_value,
            double drone_value,
            double submitted_z);

        bool TryAccept(
            const std::vector<int> &tbar,
            const std::vector<std::vector<int>> &abar,
            const std::vector<std::vector<int>> &xbar,
            const std::vector<std::vector<Node>> &per_drone_customers,
            const DroneSchedule &drone_schedule,
            const std::vector<double> &drone_values,
            const std::vector<int> &swap_counts,
            double truck_value,
            double drone_value,
            double accepted_z,
            double submitted_z);

        void HandleMIPNode();

        void RegisterDynamicUBWorkloadUserCut(double new_ub);

        bool TryAddDynamicUBWorkloadUserCut(
            int &cuts_added,
            int max_cuts);

        bool TryAddDynamicUBEnergyWorkloadUserCut(
            int &cuts_added,
            int max_cuts);

        bool IsTruckArcAllowed(Node i, Node j) const;

        void SeparateFractionalTruckConnectivityCuts(
            const std::vector<double> &trel,
            const std::vector<std::vector<double>> &xrel,
            int &cuts_added,
            int max_cuts);

        void SeparateFractionalDroneCoverCuts(
            const std::vector<std::vector<double>> &arel,
            const std::vector<double> &qrel,
            const std::vector<double> &zKrel,
            int &cuts_added,
            int max_cuts);

        bool TryAddTruckConnectivityCutsForSet(
            const std::vector<Node> &S,
            const std::vector<double> &trel,
            const std::vector<std::vector<double>> &xrel,
            int &cuts_added,
            int max_cuts);

        bool TryAddDroneCoverCutsForSet(
            int drone_id,
            const std::vector<Node> &H,
            const std::vector<std::vector<double>> &arel,
            double qrel,
            double zKrel,
            int &cuts_added,
            int max_cuts);

        void SeparateSlaveAssignAggregateDroneCuts(
            const std::vector<double> &yrel,
            double Qrel,
            double zDrel,
            int &cuts_added,
            int max_cuts);

        bool TryAddSlaveAssignAggregateDroneCutsForSet(
            const std::vector<Node> &H,
            const std::vector<double> &yrel,
            double Qrel,
            double zDrel,
            int &cuts_added,
            int max_cuts);

        void UpdateGapTelemetry(double master_lb);
        double ComputeValidGap(double valid_ub, double lower_bound) const;
        double CallbackRuntime();
    };

} // namespace bbcp
