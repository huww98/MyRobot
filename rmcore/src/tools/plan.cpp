#include <ros/ros.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace teb_local_planner;

constexpr double pi = 3.1415926535897932384626;

struct Segment
{
    int Id;
    Eigen::Vector2d lastPoint;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool distanceComp(const Eigen::Vector2d &target, const PoseSE2 &a, const PoseSE2 &b)
{
    return (target - a.position()).norm() < (target - a.position()).norm();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan");

    ros::NodeHandle nh;
    ros::NodeHandle localNh("~");
    XmlRpc::XmlRpcValue map;
    if (!nh.getParam("map", map))
    {
        ROS_FATAL("map must be set");
        ROS_BREAK();
    }

    ViaPointContainer viaPoints;

    auto segments = map["segments"];
    vector<Segment> segs;
    for (int i = 0; i < segments.size(); i++)
    {
        auto s = segments[i];
        segs.push_back(Segment{(int)s["id"]});

        auto ps = s["viaPoints"];
        for (int j = 0; j < ps.size(); j++)
        {
            auto p = ps[j];
            Eigen::Vector2d point((double)p["x"], (double)p["y"]);
            viaPoints.push_back(point);
            segs.rbegin()->lastPoint = point;
        }
    }
    auto beginPoint = *viaPoints.begin();
    auto endPoint = *viaPoints.rbegin();

    TebConfig cfg;
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> dynamic_recfg(localNh);
    dynamic_recfg.setCallback([&cfg](auto config, auto level) { cfg.reconfigure(config); });

    TebVisualizationPtr vis(new TebVisualization(localNh, cfg));

    TebOptimalPlanner planner(cfg);
    planner.setViaPoints(&viaPoints);
    planner.setVisualization(vis);
    vector<geometry_msgs::PoseStamped> initPlan;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = viaPoints[0].x();
    pose.pose.position.y = viaPoints[0].y();
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(pi / 2);
    initPlan.push_back(pose);

    for(size_t i = 1; i < viaPoints.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = viaPoints[i].x();
        pose.pose.position.y = viaPoints[i].y();
        pose.pose.position.z = 0;
        Eigen::Vector2d dir = viaPoints[i] - viaPoints[i-1];
        double yaw = atan2(dir.y(), dir.x());
        cout << yaw << endl;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        initPlan.push_back(pose);
    }
    planner.plan(initPlan);

    while(ros::ok())
    {
        planner.optimizeTEB(20, 20, true);
        double cost = planner.getCurrentCost();
        ROS_INFO("current cost: %f", cost);
        planner.visualize();
        vis->publishViaPoints(viaPoints);
    }

    ofstream fout("plan.yaml");
    YAML::Emitter out(fout);
    out << YAML::BeginMap;
    out << YAML::Key << "segments";
    out << YAML::Value << YAML::BeginSeq;

    auto &teb = planner.teb();
    int poseIdx = 0;
    int dtIdx = 0;

    for(auto &s : segs)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "id";
        out << YAML::Value << s.Id;
        out << YAML::Key << "points";
        out << YAML::Value << YAML::BeginSeq;
        int closestPoseIdx = teb.findClosestTrajectoryPose(s.lastPoint, nullptr, poseIdx);
        for(;poseIdx <= closestPoseIdx; poseIdx++)
        {
            auto &pose = teb.Pose(poseIdx);
            out << YAML::BeginMap;
            out << YAML::Key << "x";
            out << YAML::Value << pose.x();
            out << YAML::Key << "y";
            out << YAML::Value << pose.y();
            out << YAML::Key << "theta";
            out << YAML::Value << pose.theta();
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        out << YAML::Key << "timeDiff";
        out << YAML::Value << YAML::BeginSeq;
        for(;dtIdx <= closestPoseIdx - 1; dtIdx++)
        {
            out << teb.TimeDiff(dtIdx);
        }
        out << YAML::EndSeq;

        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
}
