#pragma once

#include <iostream>
#include <string>

namespace bbcp {

class Logger {
public:
    explicit Logger(bool enabled = true);
    void Info(const std::string& msg) const;
    void Warn(const std::string& msg) const;

private:
    bool enabled_ = true;
};

}  // namespace bbcp
