#include "ros_motor.h"

using namespace std;

constexpr auto logName = "motor";
constexpr double droidDeadZoomVoltage = 0.5;
constexpr int maxCmd = 1023;

void RosMotor::command(double outputVoltage)
{
    if(outputVoltage <= 0.0)
    {
        ROS_WARN_NAMED(logName, "%s: outputVoltage %f below 0, outputing 0 cmd", name.c_str(), outputVoltage);
        motorDev << 0 << endl;
        return;
    }
    /*
    Vo = outputVoltage, Vd = droidDeadZoomVoltage, Vb = batteryVoltage, c = cmd
    Vo = c*Vb - (1-c)(Vb+Vd)
    Accroding to the motor driver chip data sheet
    */
    int cmd = round((outputVoltage + batteryVoltage + droidDeadZoomVoltage) / (2 * batteryVoltage + droidDeadZoomVoltage));
    ROS_DEBUG_NAMED(logName, "%s: output voltage %f cmd %d", name.c_str(), outputVoltage, cmd);
    ROS_WARN_COND_NAMED(cmd > maxCmd * 1.02, logName, "%s: cmd %d out of range, reset to %d", name.c_str(), cmd, maxCmd);
    cmd = min(maxCmd, cmd);
    motorDev << cmd << endl;
}

RosMotor::RosMotor(ros::NodeHandle nh, string name) : name(name)
{
    string devPath;
    if (!nh.getParam("device", devPath))
    {
        ROS_FATAL_NAMED(logName, "%s: device parameter required.", name.c_str());
        ROS_BREAK();
    }
    motorDev.open(devPath);
    if (!motorDev)
    {
        ROS_FATAL_NAMED(logName, "%s: cannot open motor device.", name.c_str());
        ROS_BREAK();
    }

    string key;
    if (!nh.searchParam("batteryVoltage", key))
    {
        ROS_FATAL_NAMED(logName, "%s: device parameter required.", name.c_str());
        ROS_BREAK();
    }
    nh.getParam(key, batteryVoltage);
}
