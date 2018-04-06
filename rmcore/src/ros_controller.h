#ifndef ROS_CONTROLLER_H
#define ROS_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <tuple>
#include <Eigen/Dense>
#include "ros_motor.h"
#include "utillities/linear_interpolation.h"
#include "common_types.h"

class RosController
{
  public:
    struct PredictedTouque
    {
        double Touque;
        double DerivativeOfVelocity;
        double Variance;
    };

    RosController(ros::NodeHandle nh, std::string name, RosMotor &motor);
    // return voltage.
    double IssueCommand(double currentVelocity, double expectedTouque);
    PredictedTouque PredictTouque(double currentVelocity, double voltage);

  private:
    RosMotor *motor;
    LinearInterpolation velocityVoltageInterpolation;
    double touqueVoltageMutiplier;
    double touqueVariance;
    std::string name;

    double calcMaintainSpeedVoltage(double velocity, double &k);
    double calcMaintainSpeedVoltage(double velocity);
};

struct ControlVoltage
{
    double leftVoltage;
    double rightVoltage;
};

class RosDiffrentalController
{
  public:
    struct PredictedAcceleration
    {
        AccelerationCommand accel;
        Eigen::Matrix2d jacobianOfVelocity;
        Eigen::Matrix2d Covariance;
    };

    RosDiffrentalController(ros::NodeHandle nh, RosController &leftController, RosController &rightController);
    ControlVoltage IssueCommand(const RobotState &state, AccelerationCommand accel);
    PredictedAcceleration PredictAcceleration(const RobotState &state, ControlVoltage cmd);

  private:
    RosController *leftController, *rightController;
    double baseWidth;
    double inertiaFactor;
    Eigen::Matrix2d v2wv;
    Eigen::Matrix2d wt2a;
    Eigen::Matrix2d a2wt;

    struct WheelVelocity
    {
        double left;
        double right;
    };
    WheelVelocity calcWheelVelocity(const RobotState &state);
};

#endif
