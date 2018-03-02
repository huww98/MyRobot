#ifndef ROS_MOTOR_H
#define ROS_MOTOR_H

#include <ros/ros.h>
#include <string>
#include <fstream>

class RosMotor
{
  public:
    RosMotor(ros::NodeHandle nh, std::string name);
    void command(double outputVoltage);

  private:
    std::ofstream motorDev;
    std::string name;

    double batteryVoltage;
};

#endif
