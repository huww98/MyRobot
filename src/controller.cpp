#include <ros/ros.h>
#include <chrono>

#include "ros_imu.h"
#include "ros_encoder.h"
#include "ros_motor.h"
#include "spsc_bounded_queue.h"

SpscBoundedQueue<imu::Data> imuQueue(128);
SpscBoundedQueue<encoder::Data> leftWheelQueue(128);
SpscBoundedQueue<encoder::Data> rightWheelQueue(128);

void leftVelocityUpdated(const encoder::Data &data)
{
    leftWheelQueue.enqueue(data);
}

void rightVelocityUpdated(const encoder::Data &data)
{
    rightWheelQueue.enqueue(data);
}

void imuUpdated(const imu::Data &data)
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

    ros::NodeHandle leftMotorNode("~leftMotor");
    RosMotor leftMotor(leftMotorNode, "leftMotor");
    ros::NodeHandle rightMotorNode("~rightMotor");
    RosMotor rightMotor(rightMotorNode, "rightMotor");

    ros::waitForShutdown();

    imu.close();
}
