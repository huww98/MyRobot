#ifndef ROS_IMU_DATA_H
#define ROS_IMU_DATA_H

#include <chrono>
#include <Eigen/Dense>

namespace imu{
struct Data
{
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;

    std::chrono::steady_clock::time_point time;
};
}

#endif
