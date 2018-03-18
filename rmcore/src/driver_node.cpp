#include <ros/ros.h>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include "ros_imu.h"
#include "ros_encoder.h"
#include "ros_motor.h"
#include "ros_controller.h"
#include "spsc_bounded_queue.h"
#include "kf.h"
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
        ROS_WARN("%s is full, new data discarded.", queryName.c_str());
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

    double controlFrequency = nh.param("controlFrequency", 500.0);
    ControlScheduler scheduler(controlFrequency);

    double baseWidth;
    if(!nh.getParam("baseWidth", baseWidth))
    {
        ROS_FATAL("baseWidth parameter must be set.");
        ROS_BREAK();
    }

    RosMotor leftMotor(ros::NodeHandle("~leftMotor"), "leftMotor");
    RosMotor rightMotor(ros::NodeHandle("~rightMotor"), "rightMotor");

    RosController leftController(ros::NodeHandle("~leftController"), "leftController", leftMotor);
    RosController rightController(ros::NodeHandle("~rightController"), "rightController", rightMotor);

    RosDiffrentalController controller(ros::NodeHandle("~diffrentialController"), leftController, rightController);
    RosImu imu(imuUpdated, ros::NodeHandle("~imu"));

    RosEncoder leftEncoder(leftVelocityUpdated, ros::NodeHandle("~leftEncoder"), "leftEncoder");
    RosEncoder rightEncoder(rightVelocityUpdated, ros::NodeHandle("~rightEncoder"), "rightEncoder");

    KalmanFilter kf(baseWidth, controller);

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
                kf.UpdateLeftEncoder(eData);
            }
            while (rightWheelQueue.dequeue(eData))
            {
                kf.UpdateRightEncoder(eData);
            }
            while (imuQueue.dequeue(iData))
            {
                kf.UpdateImu(iData);
            }
        }
        else // Todo: issue control command
        {
            int skippedStep = scheduler.DoControl();
            ROS_WARN_COND(skippedStep > 0, "Skipped %d control command, maybe overloaded.", skippedStep);
        }
    }

    imu.close();
}
