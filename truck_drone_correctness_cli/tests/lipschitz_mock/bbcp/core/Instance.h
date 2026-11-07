#pragma once
#include <cstdint>
namespace bbcp {
using Node = int;
struct Instance {
    int64_t swap_time = 3;
    int64_t DroneRoundTripTime(Node i) const { return i * 5; }
};
}
