#pragma once

#include <string>
#include <vector>

#include "bbcp/core/Types.h"

namespace bbcp
{

    struct DroneSubsetCut
    {
        int drone_id = -1;
        std::vector<Node> customers;
        double rhs = 0.0;
        std::string source = "slave";
    };

    class MasterCuts
    {
    public:
        static int CountMissingCustomers(
            const std::vector<Node> &S,
            const std::vector<Node> &assigned_to_k);

        static bool IsActiveForAssignment(
            const DroneSubsetCut &cut,
            const std::vector<Node> &assigned_to_k);

        static std::string Describe(const DroneSubsetCut &cut);
    };

} // namespace bbcp
