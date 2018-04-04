#include "command_computer.h"

using namespace teb_local_planner;
using namespace std;
using namespace Eigen;

CommandComputer::CommandComputer(ros::NodeHandle nh)
{
    loadConfig(nh);
    loadSegments();
    clearOldViaPoint();
    planner.initialize(cfg);
    planner.setViaPoints(&viaPoints);
}

AccelerationCommand CommandComputer::ComputeCommand(const RobotState &state)
{
    clearOldViaPoint();
    loadSegments();

    auto &goal = planner.teb().BackPose();
    PoseSE2 start(state.Position(), state.Angle());
    geometry_msgs::Twist startVel;
    startVel.linear.x = state.Velocity();
    startVel.angular.z = state.AngularVelocity();
    startVel.angular.x = startVel.angular.y = startVel.linear.y = startVel.linear.z = 0;
    planner.plan(start, goal, &startVel);

    AccelerationCommand accel;
    double vx, vy, omega;
    planner.getVelocityCommand(vx, vy, omega);
    double dt = planner.teb().TimeDiff(0);
    accel.linear = (vx - startVel.linear.x) * 2 / dt;
    accel.angular = (omega - startVel.angular.z) * 2 / dt;
    return accel;
}

void CommandComputer::clearOldViaPoint()
{
    if(planner.teb().sizePoses() < 2)
        return;

    auto &firstVP = viaPoints[0];
    Vector2d firstPos = planner.teb().Pose(0).position();
    Vector2d secondPos = planner.teb().Pose(1).position();
    if ((firstVP - firstPos).squaredNorm() < (firstVP - secondPos).squaredNorm())
    {
        auto erasePos = viaPoints.begin();
        if((*erasePos - *(currentOnSegment->ViaPoints.rbegin())).squaredNorm() < 1e6)
        {
            currentOnSegment++;
        }
        viaPoints.erase(viaPoints.begin());
    }
}

void CommandComputer::loadSegments()
{
    auto loadedSegCount = nextSegment - currentOnSegment;
    while (loadedSegCount < 2)
    {
        auto &loadingSeg = *nextSegment;

        for (auto &p : loadingSeg.InitialPlanPoses)
        {
            planner.teb().addPose(p);
        }
        for (auto &t : loadingSeg.InitialPlanDt)
        {
            planner.teb().addTimeDiff(t);
        }

        viaPoints.push_back(*(prev(nextSegment)->ViaPoints.rbegin())); // push the last viaPoint of last segment
        for (size_t i = 0; i< loadingSeg.ViaPoints.size() - 1; i++)
        {
            viaPoints.push_back(loadingSeg.ViaPoints[i]);
        }
        nextSegment++;
    }
}

void CommandComputer::loadConfig(ros::NodeHandle nh)
{
    cfg.loadRosParamFromNodeHandle(nh);

    XmlRpc::XmlRpcValue map;
    if (!nh.getParam("map", map))
    {
        ROS_FATAL("map must be set");
        ROS_BREAK();
    }
    auto segments = map["segments"];
    for (int i = 0; i < segments.size(); i++)
    {
        auto s = segments[i];
        Segment seg;
        seg.Id = s["id"];
        auto ps = s["viaPoints"];
        for (int j = 0; j < ps.size(); j++)
        {
            auto p = ps[j];
            Eigen::Vector2d point((double)p["x"], (double)p["y"]);
            seg.ViaPoints.push_back(point);
        }
        allSegments.push_back(move(seg));
    }

    XmlRpc::XmlRpcValue plan;
    if (!nh.getParam("plan", plan))
    {
        ROS_FATAL("plan must be set");
        ROS_BREAK();
    }
    segments = map["segments"];
    for (int i = 0; i < segments.size(); i++)
    {
        auto s = segments[i];
        auto &seg = allSegments[i];
        auto ps = s["points"];
        for (int j = 0; j < ps.size(); j++)
        {
            auto p = ps[j];
            PoseSE2 pose((double)p["x"], (double)p["y"], (double)p["theta"]);
            seg.InitialPlanPoses.push_back(pose);
        }

        auto ts = s["timeDiff"];
        for (int j = 0; j < ts.size(); j++)
        {
            auto t = ts[j];
            seg.InitialPlanDt.push_back(t);
        }
    }
}
