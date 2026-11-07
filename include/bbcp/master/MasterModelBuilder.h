#pragma once

#include <string>
#include <vector>

#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"

#include "gurobi_c++.h"

namespace bbcp
{

    struct MasterVariablePlan
    {
        std::vector<std::string> assignment_truck;     // t_i
        std::vector<std::string> assignment_drone_any; // y_i
        std::vector<std::string> assignment_drone;     // a_i_k
        std::vector<std::string> truck_arcs;           // x_i_j
        std::vector<std::string> drone_time_lb;        // z_k
        std::string truck_time = "z_T";
        std::string drone_makespan = "z_D";
        std::string objective = "z";
    };

    struct MasterModelData
    {
        std::vector<GRBVar> t;              // t_i
        std::vector<GRBVar> y;              // y_i = 1 if customer i served by any drone
        std::vector<std::vector<GRBVar>> a; // a_i_k
        std::vector<std::vector<GRBVar>> x; // x_i_j
        std::vector<GRBVar> q;              // explicit mode: swap lower bound q_k
        std::vector<GRBVar> zK;             // explicit mode: z_k
        GRBVar Q;                           // slave-assign mode: aggregate swap lower bound
        std::vector<GRBVar> ordT;           // MTZ order

        GRBVar zT;
        GRBVar zD;
        GRBVar z;
    };

    class MasterModelBuilder
    {
    public:
        MasterModelBuilder(const Instance &inst, const MasterOptions &options);

        MasterVariablePlan BuildVariablePlan() const;

        MasterModelData BuildGurobiModel(GRBModel &model, double initial_ub) const;

        void Build();

    private:
        const Instance &inst_;
        const MasterOptions &options_;

        bool IsTruckArcAllowed(Node i, Node j) const;
        double TruckDirectLB(Node i) const;
        double TruckArcInclusionLB(Node i, Node j) const;

        void AddStaticPreprocessing(
            GRBModel &model,
            MasterModelData &data,
            double initial_ub) const;

        void AddStaticStrengtheningCuts(
            GRBModel &model,
            MasterModelData &data,
            double initial_ub) const;

        void ApplyBranchingAndHints(
            MasterModelData &data,
            double initial_ub) const;
    };

} // namespace bbcp
