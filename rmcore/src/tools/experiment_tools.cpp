#include <ros/ros.h>
#include <array>
#include <vector>
#include <atomic>
#include <mutex>
#include <numeric>
#include <iostream>
#include "../ros_motor.h"
#include "../ros_encoder.h"
#include "../ros_imu.h"

using namespace std;
using namespace std::chrono_literals;

array<vector<encoder::Data>, 2> rawData;
atomic<bool> collectData(false);
mutex m;

template <size_t idx>
void encoderUpdated(const encoder::Data &data)
{
    lock_guard<mutex> lock(m);
    if (!collectData)
        return;
    rawData[idx].push_back(data);
}

void enableDataCollection()
{
    lock_guard<mutex> lock(m);
    rawData = array<vector<encoder::Data>, 2>();
    collectData.store(true);
}

RosEncoder *lEncoder;
RosEncoder *rEncoder;

RosMotor *lMotor;
RosMotor *rMotor;

void cmd(double v)
{
    lMotor->command(v);
    rMotor->command(v);
}

void softStartup(double targetVoltage)
{
    for (double v = 0; v <= targetVoltage; v += 0.02) //soft startup
    {
        cmd(v);
        this_thread::sleep_for(1ms);
    }
    cmd(targetVoltage);
    this_thread::sleep_for(1s); //wait for stable
}

void softStartupLeft(double targetVoltage)
{
    for (double v = 0; v <= targetVoltage; v += 0.02) //soft startup
    {
        lMotor->command(v);
        this_thread::sleep_for(1ms);
    }
    lMotor->command(targetVoltage);
    this_thread::sleep_for(1s); //wait for stable
}

void collectEncoderData(size_t size)
{
    enableDataCollection();
    while (rawData[0].size() < size || rawData[1].size() < size) //collect data
    {
        this_thread::sleep_for(10ms);
    }
    collectData.store(false);
}

vector<imu::Data> imuRawData;
atomic<bool> collectImuData(false);
mutex imuM;

void imuUpdate(const imu::Data &data)
{
    lock_guard<mutex> lock(imuM);
    if (collectImuData)
    {
        imuRawData.push_back(data);
    }
}

void enableImuDataCollection()
{
    lock_guard<mutex> lock(imuM);
    imuRawData.clear();
    collectImuData.store(true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "experiment_tools");
    ros::NodeHandle nh;

    lEncoder = new RosEncoder(encoderUpdated<0>, ros::NodeHandle("~leftEncoder"), "leftEncoder");
    rEncoder = new RosEncoder(encoderUpdated<1>, ros::NodeHandle("~rightEncoder"), "rightEncoder");

    lMotor = new RosMotor(ros::NodeHandle("~leftMotor"), "leftMotor");
    rMotor = new RosMotor(ros::NodeHandle("~rightMotor"), "rightMotor");

    RosImu imu(imuUpdate, ros::NodeHandle("~imu"));

    ROS_DEBUG("ros init completed");

    double batteryVoltage;
    if (!ros::param::get("batteryVoltage", batteryVoltage))
        ROS_BREAK();

    ROS_INFO("init completed");

    clog << "1. velocity voltage map\n"
         << "2. acceleration test\n"
         << "3. inertia test" << endl;

    int expNum;
    cin >> expNum;
    switch (expNum)
    {
    case 1:
    {
        array<vector<double>, 2> voltageList({vector<double>{0.0}, vector<double>{0.0}}), velocityList({vector<double>{0.0}, vector<double>{0.0}});
        constexpr int step = 10;
        double stepV = batteryVoltage / step;
        for (double targetVoltage = stepV; targetVoltage <= batteryVoltage + 1e-9; targetVoltage += stepV)
        {
            voltageList[0].push_back(targetVoltage);
            voltageList[1].push_back(targetVoltage);

            softStartup(targetVoltage);
            collectEncoderData(100);
            cmd(0);

            for (int i = 0; i < 2; i++)
            {
                auto &currentRawData = rawData[i];
                double sumVel = 0.0;
                for (auto &data : currentRawData)
                    sumVel += data.velocity;
                double avgVel = sumVel / currentRawData.size();
                velocityList[i].push_back(avgVel);
            }
            this_thread::sleep_for(5s);
        }

        for (int i = 0; i < 2; i++)
        {
            cout << "velocityList: [";
            copy(velocityList[i].begin(), prev(velocityList[i].end()), ostream_iterator<double>(cout, ", "));
            cout << *velocityList[i].rbegin() << "]\n";
            cout << "voltageList: [";
            copy(voltageList[i].begin(), prev(voltageList[i].end()), ostream_iterator<double>(cout, ", "));
            cout << *voltageList[i].rbegin() << "]\n"
                 << endl;
        }
    }

    case 2:
    {
        constexpr double startV = 5.0;
        softStartup(startV);
        enableDataCollection();
        this_thread::sleep_for(0.5s);
        cmd(startV + 1.0);
        this_thread::sleep_for(1.5s);
        collectData.store(false);
        cmd(0);

        for (int i = 0; i < 2; i++)
        {
            for (auto &d : rawData[i])
            {
                cout << d.time.time_since_epoch().count() << " " << d.velocity << endl;
            }
            cout << endl;
        }
    }

    case 3:
    {
        constexpr double startV = 5.0;
        softStartupLeft(startV);
        enableImuDataCollection();
        this_thread::sleep_for(0.5s);
        lMotor->command(startV + 1.0);
        this_thread::sleep_for(1.5s);
        collectImuData.store(false);
        cmd(0);

        for (auto &d : imuRawData)
        {
            cout << d.time.time_since_epoch().count() << " " << d.angularVecocity << endl;
        }
    }
    }
}
