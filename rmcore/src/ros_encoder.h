#include <ros/ros.h>
#include <string>
#include <functional>
#include <vector>

#include "gpio.h"
#include "ros_encoder_data.h"

class RosEncoder
{
  public:
    using TickCallbackType = std::function<void(const encoder::Data &)>;
    RosEncoder(TickCallbackType tick, ros::NodeHandle nh, std::string name);
    int currentTick() const { return tickCount; }

  private:
    const std::string name;
    DigitalGpio pin;
    TickCallbackType tick;
    std::chrono::steady_clock::time_point lastTickTime;
    std::chrono::steady_clock::duration maxInterval;
    double minVelocity;
    //(meter per tick) * (count per second)
    double meterPerTickTimesCountPerSecond;
    const DigitalValue *lastState;
    int timeoutTickCount = 0;
    int tickCount = 0;
    std::vector<double> collabrateData;
    double variance;

    void pinChanged(const DigitalValue &currentValue);
    void timeouted();
};
