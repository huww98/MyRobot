#include "../state_manager.h"

using namespace std;

int main(int argc, char **argv)
{
    this_thread::sleep_for(30s);
    cerr << chrono::steady_clock::time_point::min().time_since_epoch().count() << endl;

    ros::init(argc, argv, "kf_test");
    ros::NodeHandle nh;

    RosMotor leftMotor(ros::NodeHandle("~leftMotor"), "leftMotor");
    RosMotor rightMotor(ros::NodeHandle("~rightMotor"), "rightMotor");

    RosController leftController(ros::NodeHandle("~leftController"), "leftController", leftMotor);
    RosController rightController(ros::NodeHandle("~rightController"), "rightController", rightMotor);

    RosDiffrentalController controller(ros::NodeHandle("~diffrentialController"), leftController, rightController);

    StateManager stateManager(0.1, controller);
    // KalmanFilter kf(0.1, controller, RobotState());
    this_thread::sleep_for(100ms);

    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            imu::Data data;
            data.angularVecocity = 0.1;
            data.time = chrono::steady_clock::now();
            data.var = 0.003;
            stateManager.UpdateImu(data);
            // kf.UpdateImu(data);
            this_thread::sleep_for(2ms);

            encoder::Data edata;
            edata.time = chrono::steady_clock::now();
            edata.var = 0.0001;
            edata.velocity = 0.0;

            if (j % 10 == 0)
            {
                stateManager.UpdateLeftEncoder(edata);
            }
            else if(j % 10 == 2 )
            {
                stateManager.UpdateRightEncoder(edata);
            }
            else if (j % 10 == 5)
            {
                ControlParameters params;
                params.time = chrono::steady_clock::now();
                params.command = ControlVoltage{0, 0};
                stateManager.UpdateControl(params);
            }
            else if (j % 10 == 7)
            {
                auto state = stateManager.GetPredictedState(chrono::steady_clock::now());
                cout << state.State << endl;
            }
        }
    }
    return EXIT_SUCCESS;
}
