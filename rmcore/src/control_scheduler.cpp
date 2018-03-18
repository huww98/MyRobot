#include "control_scheduler.h"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

ControlScheduler::ControlScheduler(double frequency)
    : expectedInterval(duration_cast<Duration>(1s / frequency)),
      lastControlTime(steady_clock::now()),
      scheduledTime(lastControlTime + expectedInterval),
      skippedControlStep(0)
{
}

void ControlScheduler::UpdateSchedule()
{
    while (scheduledTime + expectedInterval < steady_clock::now())
    {
        scheduledTime += expectedInterval;
        skippedControlStep++;
    }
}

int ControlScheduler::DoControl()
{
    this->UpdateSchedule();
    int skippedStep = this->skippedControlStep;
    this->skippedControlStep = 0;
    lastControlTime = scheduledTime;
    scheduledTime = lastControlTime + expectedInterval;
}
