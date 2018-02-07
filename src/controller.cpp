#include <ros/ros.h>
#include <my_robot/WheelVelocityStamped.h>
#include "ros_imu.h"

void leftVelocityUpdated(const my_robot::WheelVelocityStamped::ConstPtr &msg)
{
}

void rightVelocityUpdated(const my_robot::WheelVelocityStamped::ConstPtr &msg)
{
}

void ImuUpdated(const RosImu::Data& data)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    // auto leftVSub = n.subscribe("left_velocity", 5, leftVelocityUpdated);
    // auto RightVSub = n.subscribe("right_velocity", 5, rightVelocityUpdated);
    RosImu imu(ImuUpdated);

    ros::spin();

    imu.close();
}
