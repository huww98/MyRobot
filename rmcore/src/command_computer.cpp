#include "command_computer.h"

using namespace teb_local_planner;

CommandComputer::CommandComputer(ros::NodeHandle nh)
{
    cfg.loadRosParamFromNodeHandle(nh);
    planner.initialize(cfg);
    planner.setViaPoints(&viaPoints);
}

AccelerationCommand CommandComputer::ComputeCommand(const RobotState &state)
{

}
