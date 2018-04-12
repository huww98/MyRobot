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
#include "remote_controller.h"
#include "command_computer.h"
#include "utillities/parameters.h"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    this_thread::sleep_for(30s);

    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    double controlFrequency = 20.0;
    SearchParameter("controlFrequency", nh, controlFrequency);

    ControlScheduler scheduler(controlFrequency);

    auto baseWidth = SearchRequiredParameter<double>("baseWidth", nh);

    RosMotor leftMotor(ros::NodeHandle("~leftMotor"), "leftMotor");
    RosMotor rightMotor(ros::NodeHandle("~rightMotor"), "rightMotor");
    ROS_INFO("Motor Initialized.");

    RosController leftController(ros::NodeHandle("~leftController"), "leftController", leftMotor);
    RosController rightController(ros::NodeHandle("~rightController"), "rightController", rightMotor);
    ROS_INFO("Controller Initialized.");

    RosDiffrentalController controller(ros::NodeHandle("~diffrentialController"), leftController, rightController);
    ROS_INFO("DiffrentalController Initialized.");

    RemoteController remoteController(ros::NodeHandle("~remoteController"));
    ROS_INFO("RemoteController Initialized.");

    StateManager stateManager(baseWidth, controller);
    auto imuUpdated = [&stateManager](imu::Data d) { stateManager.UpdateImu(d); };
    auto leftVelocityUpdated = [&stateManager](encoder::Data d) { stateManager.UpdateLeftEncoder(d); };
    auto rightVelocityUpdated = [&stateManager](encoder::Data d) { stateManager.UpdateRightEncoder(d); };
    ROS_INFO("StateManager Initialized.");

    RosImu imu(imuUpdated, ros::NodeHandle("~imu"));
    ROS_INFO("IMU Initialized.");
    RosEncoder leftEncoder(leftVelocityUpdated, ros::NodeHandle("~leftEncoder"), "leftEncoder");
    RosEncoder rightEncoder(rightVelocityUpdated, ros::NodeHandle("~rightEncoder"), "rightEncoder");
    ROS_INFO("Encoder Initialized.");

    CommandComputer cmdComputer(ros::NodeHandle("~commandComputer"));

    while (ros::ok() && !cmdComputer.IsFinished())
    {
        auto time = scheduler.GetScheduledTime();
        auto nextState = stateManager.GetPredictedState(time);
        auto cmd = cmdComputer.ComputeCommand(nextState, time);
        scheduler.SleepToScheduledTime();

        ControlVoltage v;
        if(remoteController.Running())
        {
            v = controller.IssueCommand(nextState, cmd);
        }
        else
        {
            leftMotor.command(0.0);
            rightMotor.command(0.0);
            v.leftVoltage = 0.0;
            v.rightVoltage = 0.0;
        }
        ControlParameters params;
        params.command = v;
        params.time = steady_clock::now();
        stateManager.UpdateControl(params);

        ROS_DEBUG_STREAM("control cycle end. state: " << nextState.State <<
            " predicted time: " << time.time_since_epoch().count() <<
            " real time: " << params.time.time_since_epoch().count() << " command: " << cmd.vec <<
            " voltage: " << v.leftVoltage << ", " << v.rightVoltage);
    }

    imu.close();
}
