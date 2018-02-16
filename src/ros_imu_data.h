#ifndef ROS_IMU_DATA_H
#define ROS_IMU_DATA_H

#include <chrono>

namespace imu{
struct Data
{
    double angularVecocity;
    double var;
    std::chrono::steady_clock::time_point time;
};
}

#endif
