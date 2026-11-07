#pragma once

#include <chrono>

namespace bbcp {

class Timer {
public:
    Timer();
    void Reset();
    double Seconds() const;

private:
    std::chrono::steady_clock::time_point start_;
};

}  // namespace bbcp
