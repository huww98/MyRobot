#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "gpio.h"

int16_t count = 0;
const DigitalValue *lastState;
ros::Publisher encoder_pub;

void pinRising(const DigitalValue& currentValue)
{
    if (currentValue.IsHigh() && lastState->IsLow())
    {
        count++;
        std_msgs::Int16 msg;
        msg.data = count;
        ROS_DEBUG("encoder tick %d", count);
        encoder_pub.publish(msg);
    }

    lastState = &currentValue;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"encoder_driver");
    ros::NodeHandle n;
    encoder_pub = n.advertise<std_msgs::Int16>("encoder", 5);

    int encoderPinNum;
    if (!ros::param::get("~pin", encoderPinNum))
    {
        ROS_FATAL("pin parameter required.");
        return EXIT_FAILURE;
    }
    DigitalGpio encoderPin(encoderPinNum, Direction::IN);
    encoderPin.EnableISR(Edge::BOTH, pinRising);

    ros::spin();
    return 0;
}
