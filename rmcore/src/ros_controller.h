#ifndef ROS_CONTROLLER_H
#define ROS_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <tuple>
#include <Eigen/Dense>
#include "ros_motor.h"
#include "utillities/linear_interpolation.h"

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
    void IssueCommand(double currentVelocity, double expectedTouque);
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

struct ControlCommand
{
    double leftVoltage;
    double rightVoltage;
};

class RosDiffrentalController
{
  public:
    struct PredictedAcceleration
    {
        double linear;
        double angular;
        Eigen::Matrix2d jacobianOfVelocity;
        Eigen::Matrix2d Covariance;
    };

    RosDiffrentalController(ros::NodeHandle nh, RosController &leftController, RosController &rightController);
    void IssueCommand(double currentVelocity, double currentAngularVelocity, double acceleration, double angularAcceleration);
    PredictedAcceleration PredictAcceleration(double currentVelocity, double currentAngularVelocity, ControlCommand cmd);

  private:
    RosController *leftController, *rightController;
    double baseWidth;
    double inertiaFactor;

    struct WheelVelocity
    {
        double left;
        double right;
    };
    WheelVelocity calcWheelVelocity(double Velocity, double AngularVelocity);
};

#endif
