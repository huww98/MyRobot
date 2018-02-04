#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <wiringPi.h>

int16_t count = 0;
int lastState;
int encoderPin;
ros::Publisher encoder_pub;

void pinRising()
{
    int state = digitalRead(encoderPin);
    if (state && !lastState)
    {
        count++;
        std_msgs::Int16 msg;
        msg.data = count;
        ROS_DEBUG("encoder tick %d", count);
        encoder_pub.publish(msg);
    }

    lastState = state;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"encoder_driver");
    ros::NodeHandle n;
    encoder_pub = n.advertise<std_msgs::Int16>("encoder", 5);

    if(!ros::param::get("~pin", encoderPin))
    {
        ROS_ERROR("pin parameter required.");
        return 1;
    }

    wiringPiSetupSys();
    wiringPiISR(encoderPin, INT_EDGE_BOTH, pinRising);

    ros::spin();
    return 0;
}