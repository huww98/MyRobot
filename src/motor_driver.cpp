#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>
#include <fstream>

using namespace std;

const int leastCmd = 550;
const int mostCmd = 1023;

ofstream motorDev;

void cmd_recived(const std_msgs::Float32::ConstPtr& msg)
{
    auto power = msg->data; // range: 0.0 - 100.0
    int cmd;
    if(power < 0.01)
        cmd = 0;
    else
        cmd = static_cast<int>(leastCmd + power*(mostCmd-leastCmd)/100);

    ROS_DEBUG_STREAM("power: " << power << " cmd: " << cmd);
    motorDev << cmd << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"motor_driver");
    ros::NodeHandle n;

    string devPath;
    if(!ros::param::get("~device", devPath))
    {
        ROS_ERROR("device parameter required.");
        return 1;
    }
    motorDev.open(devPath);
    if(!motorDev)
    {
        ROS_ERROR("cannot open motor device.");
        return 1;
    }

    auto motor_cmd_sub = n.subscribe<std_msgs::Float32>("motor_cmd",100,cmd_recived);

    ros::spin();
}