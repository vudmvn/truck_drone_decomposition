#include "bbcp/utils/RouteUtils.h"

#include <limits>

namespace bbcp {

double ComputeRouteTime(const Instance& inst, const std::vector<Node>& route) {
    if (route.size() < 2) return 0.0;

    double value = 0.0;
    for (size_t p = 0; p + 1 < route.size(); ++p) {
        value += static_cast<double>(inst.truck_time[route[p]][route[p + 1]]);
    }
    return value;
}

std::vector<Node> RemoveCustomerFromRoute(const std::vector<Node>& route, Node customer) {
    std::vector<Node> out;
    out.reserve(route.size());
    for (Node v : route) {
        if (v != customer) out.push_back(v);
    }
    return out;
}

std::vector<Node> CheapestInsertIntoRoute(
    const Instance& inst,
    const std::vector<Node>& route,
    Node customer
) {
    if (route.size() < 2) return route;

    double best_delta = std::numeric_limits<double>::infinity();
    size_t best_pos = 1;

    for (size_t p = 0; p + 1 < route.size(); ++p) {
        Node a = route[p];
        Node b = route[p + 1];
        double delta =
            static_cast<double>(inst.truck_time[a][customer]) +
            static_cast<double>(inst.truck_time[customer][b]) -
            static_cast<double>(inst.truck_time[a][b]);

        if (delta < best_delta) {
            best_delta = delta;
            best_pos = p + 1;
        }
    }

    std::vector<Node> out = route;
    out.insert(out.begin() + static_cast<std::ptrdiff_t>(best_pos), customer);
    return out;
}

std::vector<std::vector<int>> BuildXFromRoute(
    const Instance& inst,
    const std::vector<Node>& route
) {
    const int N = inst.NumNodes();
    std::vector<std::vector<int>> x(N, std::vector<int>(N, 0));

    for (size_t p = 0; p + 1 < route.size(); ++p) {
        x[route[p]][route[p + 1]] = 1;
    }

    return x;
}

}  // namespace bbcp
