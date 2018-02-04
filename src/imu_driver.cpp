#include <ros/ros.h>
#include <limits>
#include <iostream>
#include <Eigen/Dense>
#include "imu.h"

using namespace std;
using namespace Eigen;

Imu IMU;

Matrix3d AccelCorrectMat = Matrix3d::Identity();
Vector3d AccelOffset = Vector3d::Zero();
Matrix3d GyroCorrectMat = Matrix3d::Identity();
Vector3d GyroOffset = Vector3d::Zero();

template <typename Derived>
void loadMat(XmlRpc::XmlRpcValue &param, MatrixBase<Derived> &mat)
{
    for(int i = 0; i < mat.rows(); i++)
        for(int j = 0; j < mat.cols(); j++)
            mat(i,j)=param[i][j];
}

template <typename Derived>
void loadVec(XmlRpc::XmlRpcValue &param, MatrixBase<Derived> &vec)
{
    for(int i = 0; i < vec.size(); i++)
        vec(i)=param[i];
}

void getParams()
{
    XmlRpc::XmlRpcValue am;
    if(ros::param::get("~AccelCorrectMat", am))
    {
        loadMat(am, AccelCorrectMat);
    }

    XmlRpc::XmlRpcValue ao;
    if(ros::param::get("~AccelOffset", ao))
    {
        loadVec(ao, AccelOffset);
    }

    XmlRpc::XmlRpcValue gm;
    if(ros::param::get("~GyroCorrectMat", gm))
    {
        loadMat(gm, GyroCorrectMat);
    }

    XmlRpc::XmlRpcValue go;
    if(ros::param::get("~GyroOffset", go))
    {
        loadVec(go, GyroOffset);
    }
}

void setSampleRate(double hz, uint8_t dlpfMode)
{
    double actureRate = IMU.setSampleRate(hz, dlpfMode);
    ROS_INFO("setting sampling rate to %f Hz", actureRate);
    ROS_INFO("setting digital low-pass filter to mode %d", dlpfMode);
}

void dataReady()
{
    Vector3d rawAccel(IMU.readAccelX(),IMU.readAccelY(),IMU.readAccelZ());
    Vector3d rawGyro(IMU.readGyroX(),IMU.readGyroY(),IMU.readGyroZ());
    Vector3d accel = AccelCorrectMat * (rawAccel + AccelOffset);
    Vector3d gyro = GyroCorrectMat * (rawGyro + GyroOffset);
    ROS_DEBUG("New IMU data. Accel: [%10.6f,%10.6f,%10.6f] Gyro: [%12.6f, %12.6f, %12.6f]",
        accel(0), accel(1), accel(2),
        gyro(0), gyro(1), gyro(2));
}

void enableIMU()
{
    IMU.enable();    
    ROS_INFO("IMU enabled");
}

void resetIMU()
{
    IMU.reset();
    ROS_INFO("IMU reset");    
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imu_driver");
    ros::NodeHandle n;

    ROS_DEBUG("Eigen %d.%d.%d", EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
    getParams();

    double sampleRate;
    ros::param::param<double>("~sampleRate", sampleRate, 200.0);
    int dlpfMode;
    ros::param::param<int>("~dlpfMode", dlpfMode, 0x01);

    resetIMU();
    setSampleRate(sampleRate, (uint8_t)dlpfMode);
    IMU.enableDataReadyInterrupt(dataReady);
    enableIMU();

    ros::spin();

    resetIMU();
    return 0;
}
