#include <ros/ros.h>
#include <array>
#include <vector>
#include <atomic>
#include <mutex>
#include <numeric>
#include <iostream>
#include "ros_motor.h"
#include "ros_encoder.h"

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_voltage_map");
    ros::NodeHandle nh;
    ROS_DEBUG("ros init completed");

    RosEncoder lEncoder(encoderUpdated<0>, ros::NodeHandle("~leftEncoder"), "leftEncoder");
    RosEncoder rEncoder(encoderUpdated<1>, ros::NodeHandle("~rightEncoder"), "rightEncoder");
    ROS_DEBUG("encoder init completed");

    RosMotor lMotor(ros::NodeHandle("~leftMotor"), "leftMotor");
    RosMotor rMotor(ros::NodeHandle("~rightMotor"), "rightMotor");
    ROS_DEBUG("motor init completed");

    double batteryVoltage;
    if (!ros::param::get("batteryVoltage", batteryVoltage))
        ROS_BREAK();

    ROS_INFO("init completed");

    array<vector<double>, 2> voltageList({vector<double>{0.0}, vector<double>{0.0}}), velocityList({vector<double>{0.0}, vector<double>{0.0}});
    constexpr int step = 10;
    double stepV = batteryVoltage / step;
    for (double targetVoltage = stepV; targetVoltage <= batteryVoltage + 1e-9; targetVoltage += stepV)
    {
        voltageList[0].push_back(targetVoltage);
        voltageList[1].push_back(targetVoltage);

        for (double v = 0; v <= targetVoltage; v += 0.02)//soft startup
        {
            lMotor.command(v);
            rMotor.command(v);
            this_thread::sleep_for(1ms);
        }
        lMotor.command(targetVoltage);
        rMotor.command(targetVoltage);
        this_thread::sleep_for(1s); //wait for stable

        enableDataCollection();
        while (rawData[0].size() < 100 || rawData[1].size() < 100) //collect data
        {
            this_thread::sleep_for(10ms);
        }
        collectData.store(false);
        lMotor.command(0);
        rMotor.command(0);

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
