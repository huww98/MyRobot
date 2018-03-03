#include <iostream>
#include <fstream>
#include <array>
#include <thread>
#include <chrono>
#include <atomic>
#include <numeric>
#include <iomanip>

#include "ros_encoder.h"

using namespace std;
using namespace std::chrono_literals;

array<vector<encoder::Data>, 2> rawData;
atomic<bool> collectData(false);

template <size_t idx>
void encoderUpdated(const encoder::Data &data)
{
    if (!collectData)
        return;
    rawData[idx].push_back(data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_collabrate");
    ros::NodeHandle nh;

    RosEncoder lEncoder(encoderUpdated<0>, ros::NodeHandle("~leftEncoder"), "leftEncoder");
    RosEncoder rEncoder(encoderUpdated<1>, ros::NodeHandle("~rightEncoder"), "rightEncoder");

    ofstream lw("/dev/gpiopwm/18");
    ofstream rw("/dev/gpiopwm/13");

    for (int cmd = 512; cmd <= 768; cmd += 4)
    {
        lw << cmd << endl;
        rw << cmd << endl;
        this_thread::sleep_for(1ms);
    }
    this_thread::sleep_for(1s);

    collectData.store(true);
    while (rawData[0].size() < 120)
    {
        this_thread::sleep_for(10ms);
    }
    collectData.store(false);
    lw << 0 << endl;
    rw << 0 << endl;

    array<array<vector<double>, 20>, 2> ratesData;
    cout << fixed << setprecision(4);
    for (int num = 0; num < 2; num++)
    {
        for (int i = 0; i < rawData[num].size() - 19; i++)
        {
            int calculatingIndex = i + 10;
            double sum = 0;
            for (int j = 0; j < 20; j++)
            {
                sum += rawData[num][i + j].velocity;
            }
            double avg = sum / 20;
            double currentRead = rawData[num][calculatingIndex].velocity;
            double rate = avg / currentRead;
            ratesData[num][calculatingIndex % 20].push_back(rate);
        }

        //calculate variance
        double var = 0.0;
        int count = 0;
        for (auto &rates : ratesData[num])
        {
            double avgRate = accumulate(rates.begin(), rates.end(), 0.0) / rates.size();
            cout << setw(7) << avgRate << ",";

            for (double rate : rates)
            {
                var += pow(avgRate / rate - 1, 2);
                count++;
            }
        }
        cout << endl;

        var /= count - 1;
        cout<< scientific << var << fixed << endl;
    }
}
