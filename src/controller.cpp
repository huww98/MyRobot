#include <ros/ros.h>
#include <my_robot/WheelVelocityStamped.h>
#include <chrono>

#include "ros_imu.h"
#include "spsc_bounded_queue.h"

struct WheelVelocity
{
    double velocity;
    std::chrono::steady_clock::time_point time;
};

SpscBoundedQueue<RosImu::Data> imuQueue(16);
SpscBoundedQueue<WheelVelocity> leftWheelQueue(8);
SpscBoundedQueue<WheelVelocity> rightWheelQueue(8);

void leftVelocityUpdated(const my_robot::WheelVelocityStamped::ConstPtr &msg)
{
    leftWheelQueue.enqueue(WheelVelocity());
}

void rightVelocityUpdated(const my_robot::WheelVelocityStamped::ConstPtr &msg)
{
    rightWheelQueue.enqueue(WheelVelocity());
}

void ImuUpdated(const RosImu::Data& data)
{
    imuQueue.enqueue(data);
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
