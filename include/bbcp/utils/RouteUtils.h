#pragma once

#include <vector>

#include "bbcp/core/Instance.h"

namespace bbcp {

double ComputeRouteTime(const Instance& inst, const std::vector<Node>& route);
std::vector<Node> RemoveCustomerFromRoute(const std::vector<Node>& route, Node customer);
std::vector<Node> CheapestInsertIntoRoute(const Instance& inst, const std::vector<Node>& route, Node customer);
std::vector<std::vector<int>> BuildXFromRoute(const Instance& inst, const std::vector<Node>& route);

}  // namespace bbcp
