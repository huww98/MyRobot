#include <ros/ros.h>
#include <chrono>
#include <thread>
#include "../kf.h"
#include "../ros_controller.h"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "derivative_test");
    ros::NodeHandle nh;

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

    ControlCommand cmd{3.8, 3.9};
    ControlNoise noise;
    Predictor predictor(cmd, controller, noise);

    RobotState state;
    state.State << 0.7, 0.02, 1.0, 1.0, 0.1;

    auto predictedA = controller.PredictAcceleration(state.Velocity(), state.AngularVelocity(), cmd);
    cout << predictedA.linear << ", " << predictedA.angular << endl << endl;
    cout << predictedA.jacobianOfVelocity << endl << endl;

    auto duration = 10ms;
    auto params = predictor.GetParameters(state, duration);
    cout << params.NextStateVec << endl;
    cout << params.F << endl
         << endl;

    const double delta = 1e-9;
    const double scalar = 1 / (2 * delta);
    Predictor::PredictParameters::FType computedF;
    for (int i = 0; i < state.State.size(); i++)
    {
        auto addedState = state;
        addedState.State(i) += delta;
        auto addedParams = predictor.GetParameters(addedState, duration);

        auto substractedState = state;
        substractedState.State(i) -= delta;
        auto substractedParams = predictor.GetParameters(substractedState, duration);

        auto deltaState = addedParams.NextStateVec - substractedParams.NextStateVec;
        computedF.col(i) = scalar * deltaState;
    }
    cout << computedF << endl;
}
