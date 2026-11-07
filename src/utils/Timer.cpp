#include "bbcp/utils/Timer.h"

namespace bbcp {

Timer::Timer() { Reset(); }

void Timer::Reset() {
    start_ = std::chrono::steady_clock::now();
}

double Timer::Seconds() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - start_).count();
}

}  // namespace bbcp
