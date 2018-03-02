#include <vector>
#include "ros_controller.h"

using namespace std;

auto logName = "controller";

RosController::RosController(ros::NodeHandle nh, std::string name, RosMotor &motor)
    : motor(&motor), name(name)
{
    vector<double> vecList, voltageList;
    if (!nh.getParam("vecList", vecList))
    {
        ROS_FATAL_NAMED(logName, "%s: vecList parameter must be set.", name.c_str());
        ROS_BREAK();
    }
    if (!nh.getParam("voltageList", voltageList))
    {
        ROS_FATAL_NAMED(logName, "%s: voltageList parameter must be set.", name.c_str());
        ROS_BREAK();
    }
    if (vecList.size() != voltageList.size())
    {
        ROS_FATAL_NAMED(logName, "%s: vecList and voltageList must be at the same length.", name.c_str());
        ROS_BREAK();
    }
    if (vecList.size() < 2)
    {
        ROS_FATAL_NAMED(logName, "%s: vecList must be at least 2 in length.", name.c_str());
        ROS_BREAK();
    }

    for (int i = 0; i < vecList.size(); i++)
    {
        vecVoltageMap.insert(make_pair(vecList[i], voltageList[i]));
    }

    if (!nh.getParam("touqueCmdMutiplier", touqueVoltageMutiplier))
    {
        ROS_FATAL_NAMED(logName, "%s: touqueCmdMutiplier parameter must be set.", name.c_str());
        ROS_BREAK();
    }
}

void RosController::IssueCommand(double currentVelocity, double expectedtouque)
{
    double maintainSpeedVoltage;
    auto upperPair = vecVoltageMap.upper_bound(currentVelocity);
    auto lowerPair = prev(upperPair);
    if (upperPair == vecVoltageMap.end())
    {
        maintainSpeedVoltage = lowerPair->second;
    }
    else
    {
        auto &x0 = lowerPair->first, &x1 = upperPair->first;
        auto &y0 = lowerPair->second, &y1 = upperPair->second;
        auto &x = currentVelocity;
        //线性插值
        maintainSpeedVoltage = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }

    double touqueVoltage = expectedtouque * touqueVoltageMutiplier;
    double v = maintainSpeedVoltage + touqueVoltage;
    motor->command(v);
}
