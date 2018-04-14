#include <cstdlib>

#include "ros_encoder.h"
#include "utillities/parameters.h"

using namespace std;
using std::chrono::steady_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

constexpr auto logName = "encoder";

void RosEncoder::pinChanged(const DigitalValue &currentValue)
{
    if (currentValue.IsHigh() && lastState->IsLow())
    {
        auto now = steady_clock::now();
        auto interval = now - lastTickTime;
        lastTickTime = now;
        encoder::Data data;
        if (interval > maxInterval*2)
        {
            data.time = now - maxInterval / 2; // This ensure tick goes after timeout tick.
            data.velocity = this->minVelocity / 2;
            ROS_DEBUG_STREAM_NAMED(logName, name << ": " << data.time.time_since_epoch().count() <<
                " lnterval too large, velocity: " << data.velocity << ", tick: " << tickCount);
        }
        else
        {
            data.time = now - interval / 2;
            double rawV = meterPerTickTimesCountPerSecond / interval.count();
            data.velocity = rawV * collabrateData[tickCount % collabrateData.size()];
            ROS_DEBUG_STREAM_NAMED(logName, name << ": " << data.time.time_since_epoch().count() <<
                " raw velocity: " << rawV << ", collabrated velocity: " << data.velocity << ", tick: " << tickCount);
        }

        tick(data);

        tickCount++;
    }

    lastState = &currentValue;
    timeoutTickCount = 0;
}

void RosEncoder::timeouted()
{
    auto now = steady_clock::now();
    encoder::Data data;
    data.time = now - maxInterval / 2;
    data.var = variance * pow(0.25, timeoutTickCount);
    if (timeoutTickCount == 0)
        data.velocity = minVelocity / 2;
    else
        data.velocity = 0;
    timeoutTickCount++;
    ROS_DEBUG_STREAM_NAMED(logName, name << ": " << data.time.time_since_epoch().count() <<
        " velocity: " << data.velocity << " wait timeouted");
    tick(data);
}

RosEncoder::RosEncoder(TickCallbackType tick, ros::NodeHandle nh, string name)
    : name(name), pin(GetRequiredParameter<int>("pin", nh), Direction::IN), tick(tick)
{
    auto tickPerMeter = SearchRequiredParameter<double>("encoder/tickPerMeter", nh);

    this->meterPerTickTimesCountPerSecond = (1 / tickPerMeter) * steady_clock::duration(1s).count();

    minVelocity = SearchRequiredParameter<double>("encoder/minVelocity", nh);

    duration<double> maxIntervalSecond(1 / (tickPerMeter * minVelocity));
    this->maxInterval = duration_cast<steady_clock::duration>(maxIntervalSecond);
    int maxIntervalms = duration_cast<chrono::milliseconds>(maxIntervalSecond).count();

    if (nh.getParam("collabrateData", collabrateData))
    {
        ROS_INFO_NAMED(logName, "collabrate data loaded");
    }
    else
        collabrateData.push_back(1.0);

    variance = GetRequiredParameter<double>("variance", nh);

    ROS_DEBUG_NAMED(logName, "%s: enabling ISR", name.c_str());
    pin.EnableISR(Edge::BOTH, [this](const DigitalValue &v) { this->pinChanged(v); },
                  maxIntervalms, [this] { this->timeouted(); });
    ROS_DEBUG_NAMED(logName, "%s: init complete", name.c_str());
}
