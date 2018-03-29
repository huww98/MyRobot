#include <ros/ros.h>
#include <chrono>

#include "ros_imu.h"
#include "ros_encoder.h"
#include "ros_motor.h"
#include "ros_controller.h"
#include "spsc_bounded_queue.h"
#include "state_manager.h"
#include "kf.h"
#include "control_scheduler.h"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    double controlFrequency = nh.param("controlFrequency", 500.0);
    ControlScheduler scheduler(controlFrequency);

    double baseWidth;
    if (!nh.getParam("baseWidth", baseWidth))
    {
        ROS_FATAL("baseWidth parameter must be set.");
        ROS_BREAK();
    }

    RosMotor leftMotor(ros::NodeHandle("~leftMotor"), "leftMotor");
    RosMotor rightMotor(ros::NodeHandle("~rightMotor"), "rightMotor");

    RosController leftController(ros::NodeHandle("~leftController"), "leftController", leftMotor);
    RosController rightController(ros::NodeHandle("~rightController"), "rightController", rightMotor);

    RosDiffrentalController controller(ros::NodeHandle("~diffrentialController"), leftController, rightController);

    StateManager stateManager(baseWidth, controller);
    auto imuUpdated = [&stateManager](imu::Data d) { stateManager.UpdateImu(d); };
    auto leftVelocityUpdated = [&stateManager](encoder::Data d) { stateManager.UpdateLeftEncoder(d); };
    auto rightVelocityUpdated = [&stateManager](encoder::Data d) { stateManager.UpdateRightEncoder(d); };

    RosImu imu(imuUpdated, ros::NodeHandle("~imu"));
    RosEncoder leftEncoder(leftVelocityUpdated, ros::NodeHandle("~leftEncoder"), "leftEncoder");
    RosEncoder rightEncoder(rightVelocityUpdated, ros::NodeHandle("~rightEncoder"), "rightEncoder");

    while (ros::ok())
    {
        scheduler.UpdateSchedule();
        int skippedStep = scheduler.DoControl();
        ROS_WARN_COND(skippedStep > 0, "Skipped %d control command, maybe overloaded.", skippedStep);
    }

    imu.close();
}
