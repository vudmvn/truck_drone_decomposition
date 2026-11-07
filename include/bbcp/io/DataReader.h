#pragma once

#include <cstdint>
#include <string>

#include "bbcp/core/Instance.h"

namespace bbcp {

struct ReadOptions {
    int override_n_drones = -1;
    double override_swap_minutes = -1.0;

    // Scaled energy capacity. If the data file contains BATTERY CAPACITY or
    // BATTERY CAP, that value overrides this default.
    int64_t battery_capacity = 1000;

    bool verbose = true;
};

Instance ReadInstanceFromFile(
    const std::string& filepath,
    const ReadOptions& opt = ReadOptions{}
);

class DataReader {
public:
    static Instance Read(const std::string& path);
    static Instance Read(const std::string& path, const ReadOptions& opt);
};

}  // namespace bbcp
