#include "bbcp/master/MasterCuts.h"

#include <algorithm>
#include <sstream>
#include <unordered_set>

namespace bbcp
{

    int MasterCuts::CountMissingCustomers(
        const std::vector<Node> &S,
        const std::vector<Node> &assigned_to_k)
    {
        std::unordered_set<Node> assigned(
            assigned_to_k.begin(),
            assigned_to_k.end());

        int missing = 0;
        for (Node i : S)
        {
            if (!assigned.count(i))
            {
                ++missing;
            }
        }
        return missing;
    }

    bool MasterCuts::IsActiveForAssignment(
        const DroneSubsetCut &cut,
        const std::vector<Node> &assigned_to_k)
    {
        return CountMissingCustomers(cut.customers, assigned_to_k) == 0;
    }

    std::string MasterCuts::Describe(const DroneSubsetCut &cut)
    {
        std::ostringstream oss;

        oss << "z_" << cut.drone_id
            << " + sum_{i in S} delta_i*(1-a_i_" << cut.drone_id
            << ") >= " << cut.rhs
            << "  source=" << cut.source
            << "  S={";

        for (size_t p = 0; p < cut.customers.size(); ++p)
        {
            if (p > 0)
                oss << ",";
            oss << cut.customers[p];
        }

        oss << "}";
        return oss.str();
    }

} // namespace bbcp
