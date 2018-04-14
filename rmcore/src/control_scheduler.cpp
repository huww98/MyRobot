#include <thread>
#include "control_scheduler.h"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

ControlScheduler::ControlScheduler(double frequency)
    : expectedInterval(duration_cast<DurationType>(1s / frequency)),
      lastControlTime(steady_clock::now()),
      scheduledTime(lastControlTime + expectedInterval)
{
}

void ControlScheduler::UpdateScheduledTime()
{
    auto now = steady_clock::now();
    if(scheduledTime > now || scheduledTime + expectedInterval < now)
        scheduledTime = now;

    scheduledTime += expectedInterval;
}

bool ControlScheduler::SleepToScheduledTime()
{
    auto now = steady_clock::now();
    if(scheduledTime < now)
    {
        UpdateScheduledTime();
        return false;
    }

    this_thread::sleep_until(scheduledTime);
    return true;
}
