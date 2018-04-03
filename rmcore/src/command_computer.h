#include <teb_local_planner/optimal_planner.h>
#include <ros/ros.h>

#include "common_types.h"

class CommandComputer
{
  public:
    CommandComputer(ros::NodeHandle nh);

    AccelerationCommand ComputeCommand(const RobotState &state);

  private:
    teb_local_planner::TebConfig cfg;
    teb_local_planner::TebOptimalPlanner planner;
    teb_local_planner::ViaPointContainer viaPoints;
};
