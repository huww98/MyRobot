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

template <typename T>
T SearchRequiredParameter(std::string name, ros::NodeHandle &nh)
{
    std::string key;
    if (!nh.searchParam(name, key))
    {
        ROS_FATAL("%s parameter must be set.", name.c_str());
        ROS_BREAK();
    }
    T param;
    nh.getParam(key, param);

    return param;
}

template <typename T>
void SearchParameter(std::string name, ros::NodeHandle &nh, T &param)
{
    std::string key;
    if (nh.searchParam(name, key))
    {
        nh.getParam(key, param);
    }
}

#endif
