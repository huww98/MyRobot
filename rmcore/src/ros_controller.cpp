#include <vector>
#include "ros_controller.h"

using namespace std;

auto logName = "controller";

LinearInterpolation buildVelVolInterpolation(ros::NodeHandle &nh, string name)
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

    return LinearInterpolation(velocityList, voltageList);
}

RosController::RosController(ros::NodeHandle nh, std::string name, RosMotor &motor)
    : motor(&motor), name(name), velocityVoltageInterpolation(buildVelVolInterpolation(nh, name))
{
    if (!nh.getParam("touqueVoltageMutiplier", touqueVoltageMutiplier))
    {
        ROS_FATAL_NAMED(logName, "%s: touqueVoltageMutiplier parameter must be set.", name.c_str());
        ROS_BREAK();
    }
}

double RosController::calcMaintainSpeedVoltage(double velocity, double &k)
{
    return velocityVoltageInterpolation.Y(velocity, k);
}

double RosController::calcMaintainSpeedVoltage(double velocity)
{
    double k;
    return calcMaintainSpeedVoltage(velocity, k);
}

// 力矩单位定义：在直线行驶使，两个轮子各输出一个单位力矩，可使小车获得1m/s^2的加速度。
void RosController::IssueCommand(double currentVelocity, double expectedTouque)
{
    double maintainSpeedVoltage = calcMaintainSpeedVoltage(currentVelocity);

    double touqueVoltage = expectedTouque * touqueVoltageMutiplier;
    double v = maintainSpeedVoltage + touqueVoltage;
    motor->command(v);
}

auto RosController::PredictTouque(double currentVelocity, double voltage) -> PredictedTouque
{
    double k;
    double maintainSpeedVoltage = calcMaintainSpeedVoltage(currentVelocity, k);
    double touqueVoltage = voltage - maintainSpeedVoltage;
    PredictedTouque pred;
    pred.Touque = touqueVoltage / touqueVoltageMutiplier;
    pred.DerivativeOfVelocity = -k / touqueVoltageMutiplier;
    return pred;
}

auto diffLogName = "diffController";

RosDiffrentalController::RosDiffrentalController(ros::NodeHandle nh, RosController &leftController, RosController &rightController)
    : leftController(&leftController), rightController(&rightController)
{
    string key;
    if (!nh.searchParam("baseWidth", key))
    {
        ROS_FATAL_NAMED(diffLogName, "baseWidth parameter must be set.");
        ROS_BREAK();
    }
    nh.getParam(key, baseWidth);

    // 1为所有质量都集中在两个轮子上时的转动惯量
    if (!nh.searchParam("inertia", key))
    {
        ROS_FATAL_NAMED(diffLogName, "inertia parameter must be set.");
        ROS_BREAK();
    }
    double inertia;
    nh.getParam(key, inertia);
    this->inertiaFactor = inertia * baseWidth / 2;
}

auto RosDiffrentalController::calcWheelVelocity(double Velocity, double AngularVelocity) -> WheelVelocity
{
    double leftV = Velocity - inertiaFactor * AngularVelocity;
    double rightV = Velocity + inertiaFactor * AngularVelocity;
    return WheelVelocity{leftV, rightV};
}

void RosDiffrentalController::IssueCommand(double currentVelocity, double currentAngularVelocity, double acceleration, double angularAcceleration)
{
    auto[leftV, rightV] = calcWheelVelocity(currentVelocity, currentAngularVelocity);

    double leftA = acceleration - angularAcceleration * inertiaFactor;
    double rightA = acceleration + angularAcceleration * inertiaFactor;

    leftController->IssueCommand(leftV, leftA);
    rightController->IssueCommand(rightV, rightA);
}

auto RosDiffrentalController::PredictAcceleration(double currentVelocity, double currentAngularVelocity, ControlCommand cmd)
    -> PredictedAcceleration
{
    auto[leftV, rightV] = calcWheelVelocity(currentVelocity, currentAngularVelocity);
    Eigen::Matrix2d j;
    j << 1, -inertiaFactor,
        1, inertiaFactor;

    auto leftPredictedTouque = leftController->PredictTouque(leftV, cmd.leftVoltage);
    auto rightPredictedTouque = leftController->PredictTouque(rightV, cmd.rightVoltage);

    PredictedAcceleration a;
    a.linear = (leftPredictedTouque.Touque + rightPredictedTouque.Touque) / 2;
    a.angular = (rightPredictedTouque.Touque - leftPredictedTouque.Touque) / 2 / inertiaFactor;
    // 先对轮子的速度求导
    a.jacobianOfVelocity(0, 0) = leftPredictedTouque.DerivativeOfVelocity / 2;
    a.jacobianOfVelocity(0, 1) = rightPredictedTouque.DerivativeOfVelocity / 2;
    a.jacobianOfVelocity(1, 0) = -a.jacobianOfVelocity(0, 0) / inertiaFactor;
    a.jacobianOfVelocity(1, 1) = a.jacobianOfVelocity(0, 1) / inertiaFactor;
    a.jacobianOfVelocity *= j;
    return a;
}
