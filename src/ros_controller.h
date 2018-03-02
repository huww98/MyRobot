#include <ros/ros.h>
#include <string>
#include <map>
#include "ros_motor.h"

class RosController
{
  public:
    RosController(ros::NodeHandle nh, std::string name, RosMotor &motor);
    void IssueCommand(double currentVelocity, double expectedAcceleration);

  private:
    RosMotor *motor;
    std::map<double, double> vecVoltageMap;
    double touqueVoltageMutiplier;
    std::string name;
};
