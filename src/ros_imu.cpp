#include <limits>
#include <iostream>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include "imu.h"
#include "ros_imu.h"

using namespace std;
using namespace std::chrono_literals;
using namespace Eigen;
using std::chrono::duration_cast;
using std::chrono::steady_clock;

Matrix3d RosImu::AccelCorrectMat = Matrix3d::Identity();
Vector3d RosImu::AccelOffset = Vector3d::Zero();
Matrix3d RosImu::GyroCorrectMat = Matrix3d::Identity();
Vector3d RosImu::GyroOffset = Vector3d::Zero();

constexpr auto imuLogName = "imu";

template <typename Derived>
void loadMat(XmlRpc::XmlRpcValue &param, MatrixBase<Derived> &mat)
{
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
            mat(i, j) = param[i][j];
}

template <typename Derived>
void loadVec(XmlRpc::XmlRpcValue &param, MatrixBase<Derived> &vec)
{
    for (int i = 0; i < vec.size(); i++)
        vec(i) = param[i];
}

void RosImu::getParams(const ros::NodeHandle& nh)
{
    XmlRpc::XmlRpcValue am;
    if (nh.getParam("accelCorrectMat", am))
    {
        loadMat(am, AccelCorrectMat);
    }

    XmlRpc::XmlRpcValue ao;
    if (nh.getParam("accelOffset", ao))
    {
        loadVec(ao, AccelOffset);
    }

    XmlRpc::XmlRpcValue gm;
    if (nh.getParam("gyroCorrectMat", gm))
    {
        loadMat(gm, GyroCorrectMat);
    }

    XmlRpc::XmlRpcValue go;
    if (nh.getParam("gyroOffset", go))
    {
        loadVec(go, GyroOffset);
    }
}

void RosImu::setSampleRate(double hz, uint8_t dlpfMode)
{
    double actualRate = Imu::setSampleRate(hz, dlpfMode);
    ROS_INFO_NAMED(imuLogName, "setting sampling rate to %f Hz", actualRate);
    ROS_INFO_NAMED(imuLogName, "setting digital low-pass filter to mode %d", dlpfMode);

    expectedSampleInterval = duration_cast<steady_clock::duration>(1s / actualRate);
}

constexpr double PI = 3.14159265358979323846;
constexpr double degreeToRad = PI / 180.0;

void RosImu::dataReadyHandler()
{
    auto data = readAll();
    Vector3d rawAccel = Vector3d(data.AccelX, data.AccelY, data.AccelZ);
    Vector3d rawGyro = Vector3d(data.GyroX, data.GyroY, data.GyroZ);
    imu::Data correctedData;
    correctedData.accel = AccelCorrectMat * (rawAccel + AccelOffset) * g;
    correctedData.gyro = GyroCorrectMat * (rawGyro + GyroOffset) * degreeToRad;

    auto now = steady_clock::now();
    auto measuredInterval = now - lastSampleTime;
    if (measuredInterval <= expectedSampleInterval ||
        measuredInterval > expectedSampleInterval * 1.8)
    {
        lastSampleTime = now;
        correctedData.time = now;
    }
    else
    {
        lastSampleTime += expectedSampleInterval;
        correctedData.time = lastSampleTime;
    }

    ROS_DEBUG_NAMED(imuLogName, "%lld Accel: [%10.6f,%10.6f,%10.6f] Gyro: [%10.6f, %10.6f, %10.6f]",
                    correctedData.time.time_since_epoch().count(),
                    correctedData.accel(0), correctedData.accel(1), correctedData.accel(2),
                    correctedData.gyro(0), correctedData.gyro(1), correctedData.gyro(2));

    this->dataReady(correctedData);
}

void RosImu::enableIMU()
{
    Imu::enable();
    ROS_INFO_NAMED(imuLogName, "IMU enabled");
}

void RosImu::resetIMU()
{
    Imu::reset();
    ROS_INFO_NAMED(imuLogName, "IMU reset");
}

int getInterruptPin(const ros::NodeHandle& nh)
{
    int pin;
    if(nh.getParam("interruptPin", pin))
        return pin;

    ROS_FATAL_NAMED(imuLogName, "interruptPin parameter must be set.");
    ROS_BREAK();
}

RosImu::RosImu(std::function<void(const imu::Data &)> dataReady, ros::NodeHandle nh) :
    Imu(getInterruptPin(nh)), dataReady(dataReady)
{
    ROS_DEBUG_NAMED(imuLogName, "Eigen %d.%d.%d", EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
    getParams(nh);

    double sampleRate;
    nh.param<double>("sampleRate", sampleRate, 200.0);
    int dlpfMode;
    nh.param<int>("dlpfMode", dlpfMode, 0x01);

    std::string key;
    if (nh.searchParam("g", key))
        nh.getParam(key, g);

    resetIMU();
    this_thread::sleep_for(100ms);
    enableIMU();
    this_thread::sleep_for(100ms);
    setSampleRate(sampleRate, (uint8_t)dlpfMode);
    this_thread::sleep_for(100ms);
    enableDataReadyInterrupt([this] { this->dataReadyHandler(); });
}

void RosImu::close()
{
    resetIMU();
}
