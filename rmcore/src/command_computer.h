#ifndef COMMAND_COMPUTER_H
#define COMMAND_COMPUTER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include "common_types.h"
#include "rmcore/turn.h"
#include "rmcore/visual_info.h"

struct NavigateParameters
{
    double k;
    double x_offset;
    double max_v;
};

class Navigator
{
  public:
    virtual bool Navigate(const RobotState &state, NavigateParameters &params, std::chrono::steady_clock::time_point time) = 0;
};

class FollowLine : public Navigator
{
  public:
    FollowLine(ros::NodeHandle &nh);
    bool Navigate(const RobotState &state, NavigateParameters &params, std::chrono::steady_clock::time_point time) override;

  private:
    ros::Subscriber visual_info_sub;
    void visual_info_cb(rmcore::visual_infoConstPtr msg);

    double initialized = false;
    double k = 0.0;
    double x_offset = 0.0;
};

class GoStraght : public Navigator
{
  public:
    GoStraght(double k, double goDistance);
    bool Navigate(const RobotState &state, NavigateParameters &params, std::chrono::steady_clock::time_point time) override;

  private:
    bool initialized = false;
    double keepingAngle;
    double targetDistance;
    double k;
    double goDistance;
};

class Turn : public Navigator
{
  public:
    Turn(ros::NodeHandle &nh, int direction, double k);
    bool Navigate(const RobotState &state, NavigateParameters &params, std::chrono::steady_clock::time_point time) override;

  private:
    bool initialized = false;
    bool finished = false;
    double exit_k_thre;
    double max_v;
    int direction; // -1 for left, 1 for right
    double turnSpeed;
    double startAngle;
    std::chrono::steady_clock::time_point startTime;

    double k;
};

class CommandComputer
{
  public:
    CommandComputer(ros::NodeHandle nh);
    AccelerationCommand ComputeCommand(const RobotState &state, std::chrono::steady_clock::time_point time);
    bool IsFinished() { return navigateState == NavigateState::Finished; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    ros::NodeHandle nh;

    double weight_k;
    double weight_xOffset;
    double max_centripetal_a;
    double max_v;

    Eigen::Array2d weight_p;
    Eigen::Array2d weight_i;

    double i_enable_k_threshold;
    double distance_when_start_turn;
    double distance_after_go_straight;
    double distance_after_finish;

    void turn_cb(rmcore::turnConstPtr msg);
    ros::Subscriber turn_sub;

    std::vector<int> turnList; // -1 for right, 1 for left, 0 for go straght, 2 for finish
    std::vector<int>::iterator nextTurn;

    Eigen::Array2d i;
    std::chrono::steady_clock::time_point lastTime;

    std::unique_ptr<FollowLine> followLine;
    std::unique_ptr<GoStraght> goStraght;
    std::unique_ptr<Turn> turn;
    Navigator* currentNavigator;

    enum NavigateState { FollowingLine, GoingStraght, GoingStraghtBeforeTurn, GoingStraghtBeforeFinish, Turning, Finished };
    NavigateState navigateState = NavigateState::FollowingLine;

    void transferStateTo(NavigateState state);
};

#endif
