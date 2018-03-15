#include <cstdlib>

#include "ros_encoder.h"

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
        data.time = now - interval / 2;
        if (interval > maxInterval*2)
        {
            data.velocity = this->minVelocity / 2;
            ROS_DEBUG_NAMED(logName, "%s: %lld lnterval too large, velocity: %lf, tick: %d", name.c_str(),
                            data.time.time_since_epoch().count(), data.velocity, tickCount);
        }
        else
        {
            double rawV = meterPerTickTimesCountPerSecond / interval.count();
            data.velocity = rawV * collabrateData[tickCount % collabrateData.size()];
            ROS_DEBUG_NAMED(logName, "%s: %lld raw velocity: %lf, collabrated velocity: %lf, tick: %d", name.c_str(),
                            data.time.time_since_epoch().count(), rawV, data.velocity, tickCount);
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
    if (timeoutTickCount == 0)
        data.velocity = minVelocity / 2;
    else
        data.velocity = 0;
    timeoutTickCount++;
    ROS_DEBUG_NAMED(logName, "%s: %lld velocity: %lf wait timeouted", name.c_str(),
                    data.time.time_since_epoch().count(), data.velocity);
    tick(data);
}

int getPin(const ros::NodeHandle &nh, string name)
{
    int pin;
    if (nh.getParam("pin", pin))
        return pin;

    ROS_FATAL_NAMED(logName, "%s: pin parameter must be set.", name.c_str());
    ROS_BREAK();
}

RosEncoder::RosEncoder(TickCallbackType tick, ros::NodeHandle nh, string name)
    : pin(getPin(nh, name), Direction::IN), name(name), tick(tick)
{
    double tickPerMeter;
    string key;
    if (!ros::param::search("encoder/tickPerMeter", key))
    {
        ROS_FATAL_NAMED(logName, "tickPerMeter parameter must be set.");
        ROS_BREAK();
    }
    ros::param::get(key, tickPerMeter);

    this->meterPerTickTimesCountPerSecond = (1 / tickPerMeter) * steady_clock::duration(1s).count();

    if (!ros::param::search("encoder/minVelocity", key))
    {
        ROS_FATAL_NAMED(logName, "minVelocity parameter must be set.");
        ROS_BREAK();
    }
    ros::param::get(key, minVelocity);
    duration<double> maxIntervalSecond(1 / (tickPerMeter * minVelocity));
    this->maxInterval = duration_cast<steady_clock::duration>(maxIntervalSecond);
    int maxIntervalms = duration_cast<chrono::milliseconds>(maxIntervalSecond).count();

    if (nh.getParam("collabrateData", collabrateData))
    {
        ROS_INFO_NAMED(logName, "collabrate data loaded");
    }
    else
        collabrateData.push_back(1.0);

    ROS_DEBUG_NAMED(logName, "%s: enabling ISR", name.c_str());
    pin.EnableISR(Edge::BOTH, [this](const DigitalValue &v) { this->pinChanged(v); },
                  maxIntervalms, [this] { this->timeouted(); });
    ROS_DEBUG_NAMED(logName, "%s: init complete", name.c_str());
}
