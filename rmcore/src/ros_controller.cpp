#include <vector>
#include <algorithm>
#include "ros_controller.h"
#include "utillities/parameters.h"

using namespace std;
using namespace Eigen;

auto logName = "controller";

LinearInterpolation buildVelVolInterpolation(ros::NodeHandle &nh, string name)
{
    auto velocityList = GetRequiredParameter<vector<double>>("velocityList", nh);
    auto voltageList = GetRequiredParameter<vector<double>>("voltageList", nh);
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
    : motor(&motor), velocityVoltageInterpolation(buildVelVolInterpolation(nh, name)), name(name)
{
    touqueVoltageMutiplier = GetRequiredParameter<double>("touqueVoltageMutiplier", nh);
    touqueVariance = GetRequiredParameter<double>("touqueVariance", nh);
}

double RosController::calcMaintainSpeedVoltage(double velocity, double &k)
{
    velocity = max(min(velocity, velocityVoltageInterpolation.maxX()), velocityVoltageInterpolation.minX());
    return velocityVoltageInterpolation.Y(velocity, k);
}

double RosController::calcMaintainSpeedVoltage(double velocity)
{
    double k;
    return calcMaintainSpeedVoltage(velocity, k);
}

// 力矩单位定义：在直线行驶使，两个轮子各输出一个单位力矩，可使小车获得1m/s^2的加速度。
double RosController::IssueCommand(double currentVelocity, double expectedTouque)
{
    double maintainSpeedVoltage = calcMaintainSpeedVoltage(currentVelocity);

    double touqueVoltage = expectedTouque * touqueVoltageMutiplier;
    double v = maintainSpeedVoltage + touqueVoltage;
    return motor->command(v);
}

auto RosController::PredictTouque(double currentVelocity, double voltage) -> PredictedTouque
{
    double k;
    double maintainSpeedVoltage = calcMaintainSpeedVoltage(currentVelocity, k);
    double touqueVoltage = voltage - maintainSpeedVoltage;
    PredictedTouque pred;
    pred.Touque = touqueVoltage / touqueVoltageMutiplier;
    pred.DerivativeOfVelocity = -k / touqueVoltageMutiplier;
    pred.Variance = touqueVariance;
    return pred;
}

auto diffLogName = "diffController";

RosDiffrentalController::RosDiffrentalController(ros::NodeHandle nh, RosController &leftController, RosController &rightController)
    : leftController(&leftController), rightController(&rightController)
{
    baseWidth = SearchRequiredParameter<double>("baseWidth", nh);
    // 1为所有质量都集中在两个轮子上时的转动惯量
    auto inertia = SearchRequiredParameter<double>("inertia", nh);
    this->inertiaFactor = inertia * baseWidth / 2;
    this->v2wv << 1, -baseWidth / 2,
                  1, baseWidth / 2;
    this->a2wt << 1, -inertiaFactor,
                  1, inertiaFactor;
    this->wt2a = a2wt.inverse();
}

auto RosDiffrentalController::calcWheelVelocity(const RobotState &state) -> WheelVelocity
{
    Vector2d wv = v2wv * state.V();
    return WheelVelocity{wv(0), wv(1)};
}

ControlVoltage RosDiffrentalController::IssueCommand(const RobotState &state, AccelerationCommand accel)
{
    auto v = calcWheelVelocity(state);

    Vector2d wt = a2wt * accel.vec;

    ControlVoltage voltage;
    voltage.leftVoltage = leftController->IssueCommand(v.left, wt(0));
    voltage.rightVoltage = rightController->IssueCommand(v.right, wt(1));
    return voltage;
}

auto RosDiffrentalController::PredictAcceleration(const RobotState &state, ControlVoltage cmd)
    -> PredictedAcceleration
{
    auto v = calcWheelVelocity(state);

    auto leftPredictedTouque = leftController->PredictTouque(v.left, cmd.leftVoltage);
    auto rightPredictedTouque = rightController->PredictTouque(v.right, cmd.rightVoltage);

    Vector2d wt(leftPredictedTouque.Touque, rightPredictedTouque.Touque);
    PredictedAcceleration a;
    a.accel.vec = wt2a * wt;
    DiagonalMatrix<double, 2> wv2wt(leftPredictedTouque.DerivativeOfVelocity, rightPredictedTouque.DerivativeOfVelocity);
    a.jacobianOfVelocity = wt2a * wv2wt * v2wv;

    DiagonalMatrix<double, 2> wtCov(leftPredictedTouque.Variance, rightPredictedTouque.Variance);
    a.Covariance = wt2a * wtCov * wt2a.transpose();
    return a;
}
