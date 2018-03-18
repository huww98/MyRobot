#include <vector>
#include "ros_controller.h"

using namespace std;

auto logName = "controller";

RosController::RosController(ros::NodeHandle nh, std::string name, RosMotor &motor)
    : motor(&motor), name(name)
{
    vector<double> velocityList, voltageList;
    if (!nh.getParam("velocityList", velocityList))
    {
        ROS_FATAL_NAMED(logName, "%s: velocityList parameter must be set.", name.c_str());
        ROS_BREAK();
    }
    if (!nh.getParam("voltageList", voltageList))
    {
        ROS_FATAL_NAMED(logName, "%s: voltageList parameter must be set.", name.c_str());
        ROS_BREAK();
    }
    if (velocityList.size() != voltageList.size())
    {
        ROS_FATAL_NAMED(logName, "%s: velocityList and voltageList must be at the same length.", name.c_str());
        ROS_BREAK();
    }
    if (velocityList.size() < 2)
    {
        ROS_FATAL_NAMED(logName, "%s: velocityList must be at least 2 in length.", name.c_str());
        ROS_BREAK();
    }

    for (int i = 0; i < velocityList.size(); i++)
    {
        vecVoltageMap.insert(make_pair(velocityList[i], voltageList[i]));
    }

    if (!nh.getParam("touqueCmdMutiplier", touqueVoltageMutiplier))
    {
        ROS_FATAL_NAMED(logName, "%s: touqueCmdMutiplier parameter must be set.", name.c_str());
        ROS_BREAK();
    }
}

void RosController::IssueCommand(double currentVelocity, double expectedTouque)
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

    double touqueVoltage = expectedTouque * touqueVoltageMutiplier;
    double v = maintainSpeedVoltage + touqueVoltage;
    motor->command(v);
}

auto diffLogName = "diffController";

RosDiffrentalController::RosDiffrentalController(ros::NodeHandle nh, std::string name, RosController &leftController, RosController &rightController)
    : leftController(&leftController), rightController(&rightController), name(name)
{
    string key;
    if(!nh.searchParam("baseWidth", key))
    {
        ROS_FATAL_NAMED(diffLogName, "%s: baseWidth parameter must be set.", name.c_str());
        ROS_BREAK();
    }
    nh.getParam(key, baseWidth);

    if (!nh.searchParam("inertiaFactor", key))
    {
        ROS_FATAL_NAMED(diffLogName, "%s: baseWidth parameter must be set.", name.c_str());
        ROS_BREAK();
    }
    nh.getParam(key, inertiaFactor);
}

void RosDiffrentalController::IssueCommand(double currentLeftVelocity, double currentRightVelocity, double acceleration, double angularAcceleration)
{
    double leftA = acceleration - baseWidth * angularAcceleration / 2 * inertiaFactor;
    double rightA = acceleration + baseWidth * angularAcceleration / 2 * inertiaFactor;

    leftController->IssueCommand(currentLeftVelocity, leftA);
    rightController->IssueCommand(currentRightVelocity, rightA);
}
