#include "ros_motor.h"

using namespace std;

constexpr auto logName = "motor";

void RosMotor::command(double power)
{
    int cmd = (int)(power * 1023);
    ROS_DEBUG_NAMED(logName, "%s: power %f cmd %d", name.c_str(), power, cmd);
    motorDev << cmd << endl;
}

RosMotor::RosMotor(ros::NodeHandle nh, string name): name(name)
{
    string devPath;
    if (!nh.getParam("device", devPath))
    {
        ROS_FATAL_NAMED(logName, "%s: device parameter required.", name.c_str());
        ROS_BREAK();
    }
    motorDev.open(devPath);
    if(!motorDev)
    {
        ROS_FATAL_NAMED(logName, "%s: cannot open motor device.", name.c_str());
        ROS_BREAK();
    }
}
