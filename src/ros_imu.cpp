#include <ros/ros.h>
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

Matrix3d RosImu::AccelCorrectMat = Matrix3d::Identity();
Vector3d RosImu::AccelOffset = Vector3d::Zero();
Matrix3d RosImu::GyroCorrectMat = Matrix3d::Identity();
Vector3d RosImu::GyroOffset = Vector3d::Zero();

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

void RosImu::getParams()
{
    XmlRpc::XmlRpcValue am;
    if (ros::param::get("~AccelCorrectMat", am))
    {
        loadMat(am, AccelCorrectMat);
    }

    XmlRpc::XmlRpcValue ao;
    if (ros::param::get("~AccelOffset", ao))
    {
        loadVec(ao, AccelOffset);
    }

    XmlRpc::XmlRpcValue gm;
    if (ros::param::get("~GyroCorrectMat", gm))
    {
        loadMat(gm, GyroCorrectMat);
    }

    XmlRpc::XmlRpcValue go;
    if (ros::param::get("~GyroOffset", go))
    {
        loadVec(go, GyroOffset);
    }
}

void RosImu::setSampleRate(double hz, uint8_t dlpfMode)
{
    double actualRate = Imu::setSampleRate(hz, dlpfMode);
    ROS_INFO("setting sampling rate to %f Hz", actualRate);
    ROS_INFO("setting digital low-pass filter to mode %d", dlpfMode);
}

double g;
constexpr double PI = 3.14159265358979323846;
constexpr double degreeToRad = PI / 180.0;

void RosImu::dataReadyHandler()
{
    auto data = readAll();
    Vector3d rawAccel = Vector3d(data.AccelX, data.AccelY, data.AccelZ);
    Vector3d rawGyro = Vector3d(data.GyroX, data.GyroY, data.GyroZ);
    RosImu::Data correctedData;
    correctedData.accel = AccelCorrectMat * (rawAccel + AccelOffset) * g;
    correctedData.gyro = GyroCorrectMat * (rawGyro + GyroOffset) * degreeToRad;
    ROS_DEBUG("New IMU data. Accel: [%10.6f,%10.6f,%10.6f] Gyro: [%12.6f, %12.6f, %12.6f]",
              correctedData.accel(0), correctedData.accel(1), correctedData.accel(2),
              correctedData.gyro(0), correctedData.gyro(1), correctedData.gyro(2));

    this->dataReady(correctedData);
}

void RosImu::enableIMU()
{
    Imu::enable();
    ROS_INFO("IMU enabled");
}

void RosImu::resetIMU()
{
    Imu::reset();
    ROS_INFO("IMU reset");
}

int getInterruptPin()
{
    int pin;
    if(ros::param::get("~imuInterruptPin", pin))
        return pin;

    cerr << "imuInterruptPin must be set." << endl;
    exit(EXIT_FAILURE);
}

RosImu::RosImu(std::function<void(const RosImu::Data &)> dataReady) : Imu(getInterruptPin())
{
    this->dataReady = dataReady;
    ROS_DEBUG("Eigen %d.%d.%d", EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
    getParams();

    double sampleRate;
    ros::param::param<double>("~imuSampleRate", sampleRate, 200.0);
    int dlpfMode;
    ros::param::param<int>("~imuDlpfMode", dlpfMode, 0x01);
    ros::param::param<double>("g", g, 9.8);

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
