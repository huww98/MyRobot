#include <ros/ros.h>
#include <string>
#include <fstream>

class RosMotor
{
  public:
    RosMotor(ros::NodeHandle nh, std::string name);
    void command(double power);

  private:
    std::ofstream motorDev;
    std::string name;
};
