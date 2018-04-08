#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <ros/ros.h>
#include <string>

template <typename T>
T GetRequiredParameter(std::string name, ros::NodeHandle &nh)
{
    T param;
    if(!nh.getParamCached(name, param))
    {
        ROS_FATAL("%s parameter must be set.", name.c_str());
        ROS_BREAK();
    }
    return param;
}

#endif
