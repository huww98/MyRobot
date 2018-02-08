#include <ros/ros.h>
#include <chrono>

#include "ros_imu.h"
#include "ros_encoder.h"
#include "spsc_bounded_queue.h"

struct WheelVelocity
{
    double velocity;
    std::chrono::steady_clock::time_point time;
};

SpscBoundedQueue<RosImu::Data> imuQueue(16);
SpscBoundedQueue<WheelVelocity> leftWheelQueue(8);
SpscBoundedQueue<WheelVelocity> rightWheelQueue(8);

void leftVelocityUpdated(int tick)
{
    leftWheelQueue.enqueue(WheelVelocity());
}

void rightVelocityUpdated(int tick)
{
    rightWheelQueue.enqueue(WheelVelocity());
}

void imuUpdated(const RosImu::Data& data)
{
    imuQueue.enqueue(data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    ros::NodeHandle imuNode("~imu");
    RosImu imu(imuUpdated, imuNode);

    ros::NodeHandle leftEncoderNode("~leftEncoder");
    RosEncoder leftEncoder(leftVelocityUpdated, leftEncoderNode, "leftEncoder");
    ros::NodeHandle rightEncoderNode("~rightEncoder");
    RosEncoder rightEncoder(rightVelocityUpdated, rightEncoderNode, "rightEncoder");

    ros::waitForShutdown();

    imu.close();
}
