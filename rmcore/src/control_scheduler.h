#include <chrono>

class ControlScheduler
{
  public:
    using DurationType = std::chrono::steady_clock::duration;
    using TimePointType = std::chrono::steady_clock::time_point;

    ControlScheduler(double frequency);
    TimePointType GetScheduledTime() const { return scheduledTime; }
    bool SleepToScheduledTime();

  private:
    void UpdateScheduledTime();

    const DurationType expectedInterval;
    TimePointType lastControlTime;
    TimePointType scheduledTime;
};
