#include <chrono>

class ControlScheduler
{
  public:
    using Duration = std::chrono::steady_clock::duration;
    using TimePoint = std::chrono::steady_clock::time_point;

    ControlScheduler(double frequency);
    TimePoint GetScheduledTime() const { return scheduledTime; }
    int DoControl();
    void UpdateSchedule();

  private:
    const Duration expectedInterval;
    TimePoint lastControlTime;
    TimePoint scheduledTime;
    int skippedControlStep;
}
