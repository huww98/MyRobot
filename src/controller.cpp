#include <ros/ros.h>
#include <my_robot/WheelVelocityStamped.h>
#include <sensor_msgs/Imu.h>

void leftVelocityUpdated(const my_robot::WheelVelocityStamped::ConstPtr &msg)
{
}

void rightVelocityUpdated(const my_robot::WheelVelocityStamped::ConstPtr &msg)
{
}

void ImuUpdated(const sensor_msgs::Imu::ConstPtr &msg)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    auto leftVSub = n.subscribe("left_velocity", 5, leftVelocityUpdated);
    auto RightVSub = n.subscribe("right_velocity", 5, rightVelocityUpdated);
    auto ImuSub = n.subscribe("imu", 10, ImuUpdated);

    ros::spin();
}