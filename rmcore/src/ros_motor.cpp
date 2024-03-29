#include "ros_motor.h"
#include "utillities/parameters.h"

using namespace std;

constexpr auto logName = "motor";
constexpr double droidDeadZoomVoltage = 0.5;
constexpr int maxCmd = 1023;

double RosMotor::command(double outputVoltage)
{
    if(outputVoltage < 1e-9)
    {
        ROS_WARN_COND_NAMED(outputVoltage < -1e-9, logName, "%s: outputVoltage %f below 0, outputing 0 cmd", name.c_str(), outputVoltage);
        ROS_DEBUG_NAMED(logName, "%s: output voltage: %f very small, cmd 0", name.c_str(), outputVoltage);
        motorDev << 0 << endl;
        return 0;
    }
    /*
    Vo = outputVoltage, Vd = droidDeadZoomVoltage, Vb = batteryVoltage, c = cmd/maxCmd
    Vo = c*Vb - (1-c)(Vb+Vd)
    Accroding to the motor driver chip data sheet
    */
    int cmd = round((outputVoltage + batteryVoltage + droidDeadZoomVoltage) / (2 * batteryVoltage + droidDeadZoomVoltage) * maxCmd);
    ROS_DEBUG_NAMED(logName, "%s: output voltage %f cmd %d", name.c_str(), outputVoltage, cmd);
    ROS_WARN_COND_NAMED(cmd > maxCmd * 1.02, logName, "%s: cmd %d out of range, reset to %d", name.c_str(), cmd, maxCmd);
    cmd = min(maxCmd, cmd);
    motorDev << cmd << endl;

    double c = ((double)cmd) / maxCmd;
    return c * batteryVoltage - (1-c) * (batteryVoltage + droidDeadZoomVoltage);
}

RosMotor::RosMotor(ros::NodeHandle nh, string name) : name(name)
{
    auto devPath = GetRequiredParameter<string>("device", nh);
    motorDev.open(devPath);
    if (!motorDev)
    {
        ROS_FATAL_NAMED(logName, "%s: cannot open motor device.", name.c_str());
        ROS_BREAK();
    }

    batteryVoltage = SearchRequiredParameter<double>("batteryVoltage", nh);
}
