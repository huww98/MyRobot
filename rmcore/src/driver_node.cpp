#include <ros/ros.h>
#include <chrono>

#include "ros_imu.h"
#include "ros_encoder.h"
#include "ros_motor.h"
#include "ros_controller.h"
#include "spsc_bounded_queue.h"
#include "state_manager.h"
#include "kf.h"
#include "command_computer.h"
#include "control_scheduler.h"
#include "remote_controller.h"

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

    CommandComputer cmdComputer(ros::NodeHandle("~commandComputer"));
    RemoteController remoteController(ros::NodeHandle("~remoteController"));

    StateManager stateManager(baseWidth, controller, cmdComputer.GetInitialState());
    auto imuUpdated = [&stateManager](imu::Data d) { stateManager.UpdateImu(d); };
    auto leftVelocityUpdated = [&stateManager](encoder::Data d) { stateManager.UpdateLeftEncoder(d); };
    auto rightVelocityUpdated = [&stateManager](encoder::Data d) { stateManager.UpdateRightEncoder(d); };

    RosImu imu(imuUpdated, ros::NodeHandle("~imu"));
    RosEncoder leftEncoder(leftVelocityUpdated, ros::NodeHandle("~leftEncoder"), "leftEncoder");
    RosEncoder rightEncoder(rightVelocityUpdated, ros::NodeHandle("~rightEncoder"), "rightEncoder");

    while (ros::ok())
    {
        auto time = scheduler.GetScheduledTime();
        auto nextState = stateManager.GetPredictedState(time);
        auto cmd = cmdComputer.ComputeCommand(nextState);
        scheduler.SleepToScheduledTime();

        ControlVoltage v;
        if(remoteController.Running())
            v = controller.IssueCommand(nextState, cmd);
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
        // todo: noise
        stateManager.UpdateControl(params);
    }

    imu.close();
}
