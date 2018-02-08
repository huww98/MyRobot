#include <cstdlib>

#include "ros_encoder.h"

using namespace std;

constexpr auto logName = "encoder";

void RosEncoder::pinChanged(const DigitalValue &currentValue)
{
    if (currentValue.IsHigh() && lastState->IsLow())
    {
        count++;
        ROS_DEBUG_NAMED(logName, "%s: tick %d", name.c_str(), count);
        tick(count);
    }

    lastState = &currentValue;
}

int getPin(const ros::NodeHandle &nh, string name)
{
    int pin;
    if (nh.getParam("pin", pin))
        return pin;

    ROS_FATAL_NAMED(logName, "%s: pin parameter must be set.", name.c_str());
    ROS_BREAK();
}

RosEncoder::RosEncoder(std::function<void(int)> tick, ros::NodeHandle nh, string name) :
    pin(getPin(nh, name), Direction::IN), name(name), tick(tick)
{
    pin.EnableISR(Edge::BOTH, [this](const DigitalValue &v) { this->pinChanged(v); });
}
