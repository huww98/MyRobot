#include <ros/ros.h>
#include <string>
#include <functional>

#include "gpio.h"
#include "ros_encoder_data.h"

class RosEncoder
{
  public:
    RosEncoder(std::function<void(const encoder::Data &)> tick, ros::NodeHandle nh, std::string name);

  private:
    const std::string name;
    DigitalGpio pin;
    std::function<void(const encoder::Data &)> tick;
    std::chrono::steady_clock::time_point lastTickTime;
    std::chrono::steady_clock::duration maxInterval;
    double minVelocity;
    //(meter per tick) * (count per second)
    double meterPerTickTimesCountPerSecond;
    const DigitalValue *lastState;
    int timeoutTickCount = 0;

    void pinChanged(const DigitalValue &currentValue);
    void timeouted();
};
