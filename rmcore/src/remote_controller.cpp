#include "remote_controller.h"

using namespace ros;
using namespace std;

bool RemoteController::run_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    _running = req.data;
    ROS_INFO_COND(_running, "Now Running");
    ROS_INFO_COND(!_running, "Now Stoping");
    res.success = true;
    return true;
}

RemoteController::RemoteController(NodeHandle nh)
{
    runService = nh.advertiseService("run", &RemoteController::run_cb, this);
}
