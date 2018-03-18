#include <ros/ros.h>
#include <functional>
#include <Eigen/Dense>
#include "imu.h"
#include "ros_imu_data.h"

class RosImu: public Imu
{
  public:
    RosImu(std::function<void(const imu::Data &)> dataReady, ros::NodeHandle nh);
    void close();

  private:
    std::function<void(const imu::Data &)> dataReady;
    void dataReadyHandler();
    void getParams(const ros::NodeHandle&);
    void setSampleRate(double hz, uint8_t dlpfMode);
    void enableIMU();
    void resetIMU();

    Eigen::Matrix3d GyroCorrectMat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d GyroOffset = Eigen::Vector3d::Zero();

    double g = 9.8;
    std::chrono::steady_clock::time_point lastSampleTime;
    std::chrono::steady_clock::duration expectedSampleInterval;
};
