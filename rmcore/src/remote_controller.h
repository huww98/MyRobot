#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class RemoteController
{
  public:
    RemoteController(ros::NodeHandle nh);
    bool Running() const { return _running; }

  private:
    bool _running = false;
    ros::ServiceServer runService;

    bool run_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
};
