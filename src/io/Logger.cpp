#include "bbcp/io/Logger.h"

namespace bbcp {

Logger::Logger(bool enabled) : enabled_(enabled) {}

void Logger::Info(const std::string& msg) const {
    if (enabled_) std::cout << "[INFO] " << msg << "\n";
}

void Logger::Warn(const std::string& msg) const {
    if (enabled_) std::cerr << "[WARN] " << msg << "\n";
}

}  // namespace bbcp
