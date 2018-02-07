#include <functional>
#include <Eigen/Dense>
#include "imu.h"

class RosImu: public Imu
{
  public:
    struct Data
    {
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
    };
    RosImu(std::function<void(const Data &)> dataReady);
    void close();

  private:
    std::function<void(const Data &)> dataReady;
    void dataReadyHandler();
    void getParams();
    void setSampleRate(double hz, uint8_t dlpfMode);
    void enableIMU();
    void resetIMU();

    static Eigen::Matrix3d AccelCorrectMat;
    static Eigen::Vector3d AccelOffset;
    static Eigen::Matrix3d GyroCorrectMat;
    static Eigen::Vector3d GyroOffset;
};
