#include <teb_local_planner/optimal_planner.h>
#include <ros/ros.h>
#include <vector>
#include "common_types.h"

struct Segment
{
    int Id;
    std::vector<Eigen::Vector2d> ViaPoints;
    std::vector<teb_local_planner::PoseSE2> InitialPlanPoses;
    std::vector<double> InitialPlanDt;
};

class CommandComputer
{
  public:
    CommandComputer(ros::NodeHandle nh);

    AccelerationCommand ComputeCommand(const RobotState &state);

  private:
    void loadSegments();
    void loadConfig(ros::NodeHandle nh);
    void clearOldViaPoint();

    std::vector<Segment> allSegments;
    std::vector<Segment>::iterator currentOnSegment = allSegments.begin();
    std::vector<Segment>::iterator nextSegment = allSegments.begin();

    teb_local_planner::TebConfig cfg;
    teb_local_planner::TebOptimalPlanner planner;
    teb_local_planner::ViaPointContainer viaPoints;
};
