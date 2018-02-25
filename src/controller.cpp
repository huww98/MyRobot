#include <ros/ros.h>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include "ros_imu.h"
#include "ros_encoder.h"
#include "ros_motor.h"
#include "spsc_bounded_queue.h"
#include "kf.h"
#include "kf_step.h"
#include "control_scheduler.h"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

bool hasNewData = false;
mutex m;
condition_variable cv;

SpscBoundedQueue<imu::Data> imuQueue(32);
SpscBoundedQueue<encoder::Data> leftWheelQueue(8);
SpscBoundedQueue<encoder::Data> rightWheelQueue(8);

template <typename dataType>
void enqueueNewData(SpscBoundedQueue<dataType> &dataQuery, const dataType &data, string queryName)
{
    if (dataQuery.enqueue(data))
    {
        {
            lock_guard<mutex> lk(m);
            hasNewData = true;
        }
        cv.notify_one();
    }
    else
        ROS_WARN("%s is full, new data discarded.", queryName);
}

void leftVelocityUpdated(const encoder::Data &data)
{
    enqueueNewData(leftWheelQueue, data, "leftWheelQueue");
}

void rightVelocityUpdated(const encoder::Data &data)
{
    enqueueNewData(rightWheelQueue, data, "rightWheelQueue");
}

void imuUpdated(const imu::Data &data)
{
    enqueueNewData(imuQueue, data, "imuQueue");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    double controlFrequency;
    nh.param("controlFrequency", controlFrequency, 500.0);
    ControlScheduler scheduler(controlFrequency);

    KalmanFilter kf;

    ros::NodeHandle leftMotorNode("~leftMotor");
    RosMotor leftMotor(leftMotorNode, "leftMotor");
    ros::NodeHandle rightMotorNode("~rightMotor");
    RosMotor rightMotor(rightMotorNode, "rightMotor");

    ros::NodeHandle imuNode("~imu");
    RosImu imu(imuUpdated, imuNode);

    ros::NodeHandle leftEncoderNode("~leftEncoder");
    RosEncoder leftEncoder(leftVelocityUpdated, leftEncoderNode, "leftEncoder");
    ros::NodeHandle rightEncoderNode("~rightEncoder");
    RosEncoder rightEncoder(rightVelocityUpdated, rightEncoderNode, "rightEncoder");

    while (ros::ok())
    {
        scheduler.UpdateSchedule();
        bool processData;
        {
            unique_lock<mutex> lk(m);
            processData = cv.wait_until(lk, scheduler.GetScheduledTime(), [] { return hasNewData; });
            hasNewData = false;
        }

        if (processData)
        {
            encoder::Data eData;
            imu::Data iData;
            while (leftWheelQueue.dequeue(eData))
            {
                kf.Update(KalmanFilter::UpdateStepPtr(new LeftEncoderUpdateStep(eData)), false);
            }
            while (rightWheelQueue.dequeue(eData))
            {
                kf.Update(KalmanFilter::UpdateStepPtr(new RightEncoderUpdateStep(eData)), false);
            }
            while (imuQueue.dequeue(iData))
            {
                kf.Update(KalmanFilter::UpdateStepPtr(new GyroUpdateStep(iData)), false);
            }
        }
        else // issue control command
        {
            int skippedStep = scheduler.DoControl();
            ROS_WARN_COND(skippedStep > 0, "Skipped %d control command, maybe overloaded.", skippedStep);
        }
    }

    imu.close();
}
