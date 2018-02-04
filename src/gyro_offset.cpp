#include "imu.h"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

using namespace std;
using namespace Eigen;

Vector3d gyroSum = Vector3d::Zero();
int readCount = 0;
const int maxReadCount = 8192;
Imu imu;

void dataReady()
{
    if (readCount >= maxReadCount)
        return;

    gyroSum += Vector3d(imu.readGyroX(), imu.readGyroY(), imu.readGyroZ());
    readCount++;
}

int main()
{
    using namespace std::chrono_literals;

    imu.reset();
    imu.setSampleRate(200);
    imu.enableDataReadyInterrupt(dataReady);
    imu.enable();

    while (readCount < maxReadCount)
        std::this_thread::sleep_for(10ms);

    Vector3d offset = -gyroSum / readCount;
    cout << setprecision(7) ;
    cout << "[" << setw(10) << offset(0)
        << "," << setw(10) << offset(1)
        << "," << setw(10) << offset(2) << "]" << endl;

    imu.reset();
}
