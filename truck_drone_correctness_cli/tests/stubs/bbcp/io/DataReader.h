#pragma once
#include <string>
#include "bbcp/core/Instance.h"
namespace bbcp{
struct ReadOptions{int override_n_drones=-1;double override_swap_minutes=-1;};
inline Instance ReadInstanceFromFile(const std::string&,const ReadOptions& o){Instance i;i.n_drones=o.override_n_drones>0?o.override_n_drones:4;return i;}
}
