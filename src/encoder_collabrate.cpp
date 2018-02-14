#include <queue>
#include <array>

#include "ros_encoder.h"

using namespace std;

array<queue<encoder::Data>, 2> dataQueues;

template <size_t idx>
void encoderUpdated(const encoder::Data &data)
{
    dataQueues[idx].push(data)
}

int main()
{
    RosEncoder lEncoder(encoderUpdated<0>, ros::NodeHandle("~leftEncoder"), "leftEncoder");
    RosEncoder rEncoder(encoderUpdated<1>, ros::NodeHandle("~rightEncoder"), "rightEncoder");
}
