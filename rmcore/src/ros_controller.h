#include <ros/ros.h>
#include <string>
#include <map>
#include "ros_motor.h"

class RosController
{
  public:
    RosController(ros::NodeHandle nh, std::string name, RosMotor &motor);
    void IssueCommand(double currentVelocity, double expectedTouque);

  private:
    RosMotor *motor;
    std::map<double, double> vecVoltageMap;
    double touqueVoltageMutiplier;
    std::string name;
};

class RosDiffrentalController
{
  public:
    RosDiffrentalController(ros::NodeHandle nh, std::string name, RosController &leftController, RosController &rightController);
    void IssueCommand(double currentLeftVelocity, double currentRightVelocity, double acceleration, double angularAcceleration);

  private:
    RosController *leftController, *rightController;
    double baseWidth;
    double inertiaFactor;
    std::string name;
}
