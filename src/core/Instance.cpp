#include "bbcp/core/Instance.h"

namespace bbcp {

int Instance::NumNodes() const {
    return n + 2;
}

bool Instance::IsCustomer(Node i) const {
    return i >= 1 && i <= n;
}

bool Instance::IsDroneFeasible(Node i) const {
    return IsCustomer(i) &&
           i < static_cast<int>(drone_feasible.size()) &&
           drone_feasible[i];
}

int64_t Instance::DroneRoundTripTime(Node i) const {
    if (i >= 0 && i < static_cast<int>(drone_roundtrip_time.size()) &&
        drone_roundtrip_time[i] > 0) {
        return drone_roundtrip_time[i];
    }

    if (i >= 0 && i < static_cast<int>(drone_time.size()) &&
        !drone_time.empty() &&
        0 < static_cast<int>(drone_time.size()) &&
        i < static_cast<int>(drone_time[0].size())) {
        return drone_time[0][i] + drone_time[i][0];
    }

    return 0;
}

std::vector<Node> Instance::DroneCustomers() const {
    std::vector<Node> out;
    for (Node i = 1; i <= n; ++i) {
        if (IsDroneFeasible(i)) {
            out.push_back(i);
        }
    }
    return out;
}

}  // namespace bbcp
