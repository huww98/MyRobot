#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <limits>
#include <cmath>
#include <vector>

#include "../imu.h"

using namespace std;
using namespace std::chrono_literals;
using namespace Eigen;

Vector3d gyroSum;
atomic<int> readCount;
vector<Vector3d> reads;
atomic<int> maxReadCount;
Imu imu(14);

void dataReady()
{
    if (readCount >= maxReadCount)
        return;

    auto data = imu.readAll();
    Vector3d vec(data.GyroX, data.GyroY, data.GyroZ);
    gyroSum += vec;
    reads.push_back(vec);
    readCount++;
}

void waitForData(int count)
{
    readCount.store(numeric_limits<int>::max()); // Ensure data is not recorded until everything is set.
    gyroSum = Vector3d::Zero();
    reads.clear();
    maxReadCount.store(count);
    readCount.store(0);

    while (readCount < maxReadCount)
        this_thread::sleep_for(10ms);
}

void outputData(string path)
{
    ofstream file(path);
    for(auto& data : reads)
    {
        file << data.transpose() << endl;
    }
}

int main()
{
    clog << "本程序将激活电机，请将小车放在开阔地面。^C终止" << endl;

    imu.reset();
    this_thread::sleep_for(100ms);
    imu.enable();
    imu.setSampleRate(500, 0x00);
    imu.enableDataReadyInterrupt(dataReady);

    this_thread::sleep_for(2s);
    clog << "正在收集静止数据，预计10s" << endl;
    waitForData(5000);
    outputData("stop.log");
    Vector3d offset = -gyroSum / readCount;
    cout << "gyroOffset: [" << setw(10) << offset(0)
         << "," << setw(10) << offset(1)
         << "," << setw(10) << offset(2) << "]" << endl;

    const int expectCmd = 800;
    clog << "进行旋转，收集数据，右轮输出" << expectCmd << "，预计15s" << endl;
    ofstream rightWheel("/dev/gpiopwm/13");

    for (int cmd = 512; cmd <= expectCmd; cmd += 4)
    {
        rightWheel << cmd << endl;
        this_thread::sleep_for(4ms);
    }
    this_thread::sleep_for(2.5s); // 等待稳定

    waitForData(5000);
    outputData("turning.log");
    Vector3d zAxisData = gyroSum / readCount + offset;
    clog << "measured angular velocity:\n"
         << zAxisData << endl;

    double thetaY = atan(zAxisData(0) / zAxisData(2));
    clog << "thetaY: " << thetaY << endl;
    Matrix3d correctMat1 = AngleAxisd(-thetaY, Vector3d(0, 1, 0)).toRotationMatrix();
    Vector3d zAxisData1 = correctMat1 * zAxisData;

    clog << "x corrected angular velocity:\n"
         << zAxisData1 << endl;

    double thetaX = atan(zAxisData1(1) / zAxisData1(2));
    clog << "thetaX: " << thetaX << endl;
    Matrix3d correctMat2 = AngleAxisd(thetaX, Vector3d(1, 0, 0)).toRotationMatrix();
    Vector3d zAxisData2 = correctMat2 * zAxisData1;

    clog << "corrected angular velocity:\n"
         << zAxisData2 << endl;

    Matrix3d correctMat = correctMat2 * correctMat1;
    cout << "gyroCorrectMat:\n" << correctMat << endl;

    Matrix3d cov = Matrix3d::Zero();
    for (auto &data : reads)
    {
        Vector3d diffData = correctMat * (data + offset) - zAxisData2;
        cov += diffData * diffData.transpose();
    }
    cov /= readCount;
    cout << "gyroCovMat:\n" << cov << endl;

    imu.reset();
    rightWheel << 0 << endl;
}
