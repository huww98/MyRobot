#include "state_manager.h"

using namespace std;

StateManager::StateManager(double baseWidth, RosDiffrentalController &controller, const RobotState &initState)
    : imuQueue(32), leftEncoderQueue(8), rightEncoderQueue(8), kf(baseWidth, controller, initState),
      insertThread([this] { this->insertWorker(); })
{
}

template <typename dataType>
void StateManager::enqueueNewData(SpscBoundedQueue<dataType> &dataQuery, const dataType &data, string queryName)
{
    if (dataQuery.enqueue(data))
    {
        {
            lock_guard<mutex> lk(m);
            hasNewData = true;
        }
        cv.notify_one();
    }
    else
        ROS_WARN("%s is full, new data discarded.", queryName.c_str());
}

void StateManager::UpdateLeftEncoder(const encoder::Data &data)
{
    enqueueNewData(leftEncoderQueue, data, "left encoder queue");
}
void StateManager::UpdateRightEncoder(const encoder::Data &data)
{
    enqueueNewData(rightEncoderQueue, data, "right encoder queue");
}
void StateManager::UpdateImu(const imu::Data &data)
{
    enqueueNewData(imuQueue, data, "imu queue");
}

void StateManager::insertWorker()
{
    while (true)
    {
        {
            std::unique_lock<std::mutex> lk(m);
            cv.wait(lk, [this] { return hasNewData || !running; });
            if (!running)
                break;
            hasNewData = false;
        }

        encoder::Data eData;
        imu::Data iData;
        while (leftEncoderQueue.dequeue(eData))
        {
            lock_guard<mutex> lk(insertMutex);
            kf.UpdateLeftEncoder(eData);
        }
        while (rightEncoderQueue.dequeue(eData))
        {
            lock_guard<mutex> lk(insertMutex);
            kf.UpdateRightEncoder(eData);
        }
        while (imuQueue.dequeue(iData))
        {
            lock_guard<mutex> lk(insertMutex);
            kf.UpdateImu(iData);
        }
    }
}

void StateManager::UpdateControl(const ControlParameters &params)
{
    lock_guard<mutex> lk(insertMutex);
    kf.Predict(params);
}

const RobotState &StateManager::GetPredictedState(KalmanFilter::TimePointType time)
{
    lock_guard<mutex> lk(insertMutex);
    kf.SetPredictEndTime(time);
    return kf.GetLatestState();
}

StateManager::~StateManager()
{
    {
        lock_guard<mutex> lk(m);
        running = false;
    }
    cv.notify_one();
    insertThread.join();
}
