#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "bbcp/core/Types.h"

namespace bbcp {

constexpr int64_t INF_ENERGY = static_cast<int64_t>(1e15);

struct Coord {
    double x = 0.0;
    double y = 0.0;
};

struct Instance {
    std::string name;

    int n = 0;
    int n_drones = 0;

    Node start_depot = 0;
    Node end_depot = 0;

    int64_t battery_capacity = 0;
    int64_t swap_time = 0;

    // Legacy data fields from the previous implementation.
    double drone_cap = 0.0;
    double truck_speed = 0.0;
    double drone_speed = 0.0;

    std::vector<Coord> coords;
    std::vector<double> demand;

    std::vector<std::vector<int64_t>> truck_time;
    std::vector<std::vector<int64_t>> drone_time;

    // For the current decomposed solver, each drone task is treated as a
    // depot -> customer -> depot sortie. This vector stores that round-trip
    // time for each customer.
    std::vector<int64_t> drone_roundtrip_time;

    std::vector<int64_t> energy;
    std::vector<bool> drone_feasible;

    int NumNodes() const;
    bool IsCustomer(Node i) const;
    bool IsDroneFeasible(Node i) const;
    int64_t DroneRoundTripTime(Node i) const;

    std::vector<Node> DroneCustomers() const;
};

}  // namespace bbcp
