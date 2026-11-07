#include "bbcp/core/Instance.h"
#include "bbcp/slave/SingleDroneOrderingSlave.h"

int main() {
    bbcp::Instance inst;
    inst.n = 1;
    inst.n_drones = 1;
    inst.start_depot = 0;
    inst.end_depot = 2;
    inst.battery_capacity = 10;
    inst.swap_time = 1;
    inst.truck_time = {{0,1,1},{1,0,1},{1,1,0}};
    inst.drone_roundtrip_time = {0,5,0};
    inst.energy = {0,5,0};
    inst.drone_feasible = {false,true,false};

    bbcp::SingleDroneOrderingSlave slave(inst);
    bbcp::SlaveOptions opt;
    auto res = slave.Solve({1}, opt);
    return (res.feasible && res.objective == 5) ? 0 : 1;
}
