#include <condition_variable>
#include <mutex>
#include <thread>
#include "kf.h"
#include "spsc_bounded_queue.h"

class StateManager
{
  private:
    SpscBoundedQueue<imu::Data> imuQueue;
    SpscBoundedQueue<encoder::Data> leftEncoderQueue;
    SpscBoundedQueue<encoder::Data> rightEncoderQueue;
    KalmanFilter kf;

    bool hasNewData = false;
    std::mutex m;
    std::condition_variable cv;

    bool running = true;
    std::thread insertThread;
    void insertWorker();
    std::mutex insertMutex;

    template <typename dataType>
    void enqueueNewData(SpscBoundedQueue<dataType> &dataQuery, const dataType &data, std::string queryName);

  public:
    StateManager(double baseWidth, RosDiffrentalController &controller);
    void UpdateLeftEncoder(const encoder::Data &data);
    void UpdateRightEncoder(const encoder::Data &data);
    void UpdateImu(const imu::Data &data);
    void UpdateControl(const ControlParameters &params);
    const RobotState &GetPredictedState(KalmanFilter::TimePointType time);
    ~StateManager();
};
