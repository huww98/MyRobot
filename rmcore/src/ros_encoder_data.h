#ifndef ROS_ENCODER_DATA
#define ROS_ENCODER_DATA

#include <chrono>

namespace encoder{
struct Data
{
    double velocity;
    double var;
    std::chrono::steady_clock::time_point time;
};
}

#endif
