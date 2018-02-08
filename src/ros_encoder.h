#include <ros/ros.h>
#include <string>
#include <functional>

#include "gpio.h"

class RosEncoder
{
  public:
    RosEncoder(std::function<void(int)> tick, ros::NodeHandle nh, std::string name);

  private:
    const std::string name;
    DigitalGpio pin;
    std::function<void(int)> tick;
    int count = 0;
    const DigitalValue *lastState;

    void pinChanged(const DigitalValue &currentValue);
};
