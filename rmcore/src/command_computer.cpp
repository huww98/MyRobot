#include "command_computer.h"
#include "utillities/parameters.h"
#include "utillities/constants.h"

using namespace std;
using namespace std::chrono;

FollowLine::FollowLine(ros::NodeHandle &nh)
{
    visual_info_sub = nh.subscribe("visual_info", 1, &FollowLine::visual_info_cb, this);
}

bool FollowLine::Navigate(const RobotState &state, NavigateParameters &params, steady_clock::time_point time)
{
    params.k = k;
    params.x_offset = x_offset;
    params.max_v = 1e99;
    return true;
}

void FollowLine::visual_info_cb(rmcore::visual_infoConstPtr msg)
{
    x_offset = msg->x_offset;
    k = msg->k;
}

GoStraght::GoStraght(double k, double goDistance): k(k), goDistance(goDistance) {}

bool GoStraght::Navigate(const RobotState &state, NavigateParameters &params, steady_clock::time_point time)
{
    if (!initialized)
    {
        keepingAngle = state.Angle() + atan(k);
        targetDistance = state.Distance() + goDistance;
        initialized = true;
    }
    params.k = tan(keepingAngle - state.Angle());
    params.x_offset = 0;
    params.max_v = 1e99;
    return state.Distance() < targetDistance;
}

Turn::Turn(ros::NodeHandle &nh, int direction, double k): k(k), direction(direction)
{
    exit_k_thre = GetRequiredParameter<double>("exit_k_threshold", nh);
    max_v = GetRequiredParameter<double>("turning_max_v", nh);
    turnSpeed = GetRequiredParameter<double>("turn_speed", nh);
}

bool Turn::Navigate(const RobotState &state, NavigateParameters &params, steady_clock::time_point time)
{
    if (!initialized)
    {
        startAngle = state.Angle() + atan(k);
        startTime = time;
        initialized = true;
    }

    double turnedAngle;
    if(!finished)
    {
        double t = duration_cast<chrono::duration<double>>(time-startTime).count();
        turnedAngle = startAngle + t * turnSpeed;
        finished = turnedAngle > PI / 2;
    }

    if (finished)
    {
        turnedAngle = PI / 2;
    }
    params.k = tan(startAngle + direction * turnedAngle - state.Angle());
    params.x_offset = 0;
    params.max_v = max_v;
    return finished && abs(params.k) < exit_k_thre;
}

CommandComputer::CommandComputer(ros::NodeHandle nh): nh(nh)
{
    weight_k = GetRequiredParameter<double>("weight_k", nh);
    weight_xOffset = GetRequiredParameter<double>("weight_xOffset", nh);
    max_centripetal_a = GetRequiredParameter<double>("max_centripetal_a", nh);
    max_v = GetRequiredParameter<double>("max_v", nh);
    weight_p(0) = GetRequiredParameter<double>("weight_v_p", nh);
    weight_p(1) = GetRequiredParameter<double>("weight_omiga_p", nh);
    weight_i(0) = GetRequiredParameter<double>("weight_v_i", nh);
    weight_i(1) = GetRequiredParameter<double>("weight_omiga_i", nh);
    i_enable_k_threshold = GetRequiredParameter<double>("i_enable_k_threshold", nh);
    distance_when_start_turn = GetRequiredParameter<double>("distance_when_start_turn", nh);
    distance_after_go_straight = GetRequiredParameter<double>("distance_after_go_straight", nh);
    distance_after_finish = GetRequiredParameter<double>("distance_after_finish", nh);
    turnList = GetRequiredParameter<vector<int>>("turnList", nh);

    nextTurn = turnList.begin();
    turn_sub = nh.subscribe("turn", 1, &CommandComputer::turn_cb, this);

    followLine.reset(new FollowLine(nh));
    currentNavigator = followLine.get();
}

AccelerationCommand CommandComputer::ComputeCommand(const RobotState &state, steady_clock::time_point time)
{
    ROS_ASSERT(!IsFinished());

    double t = duration_cast<chrono::duration<double>>(time-lastTime).count();
    lastTime = time;

    NavigateParameters params;
    bool navigateFinished = currentNavigator->Navigate(state, params, time);

    Eigen::Vector2d setpoint;
    setpoint(1) = params.k * weight_k + params.x_offset * weight_xOffset;
    setpoint(0) = min({max_centripetal_a / setpoint(1), max_v, params.max_v});

    Eigen::Vector2d error = setpoint - state.V();
    if(abs(params.k) < i_enable_k_threshold)
    {
        i += error * t;
    }

    AccelerationCommand a;
    a.vec = error.cwiseProduct(weight_p) + i.cwiseProduct(weight_i);

    if (navigateFinished)
    {
        switch (navigateState)
        {
        case NavigateState::GoingStraght:
            goStraght.reset(nullptr);
            transferStateTo(NavigateState::FollowingLine);
            break;
        case NavigateState::Turning:
            turn.reset(nullptr);
            transferStateTo(NavigateState::FollowingLine);
            break;
        case NavigateState::GoingStraghtBeforeTurn:
            goStraght.reset(nullptr);
            turn.reset(new Turn(nh, *nextTurn, params.k));
            nextTurn++;
            transferStateTo(NavigateState::Turning);
            break;
        case NavigateState::GoingStraghtBeforeFinish:
            goStraght.reset(nullptr);
            transferStateTo(NavigateState::Finished);
        default:
            ROS_BREAK();
        }
    }

    return a;
}

void CommandComputer::turn_cb(rmcore::turnConstPtr msg)
{
    if (navigateState != NavigateState::FollowingLine)
    {
        return;
    }

    if (nextTurn == turnList.end())
    {
        ROS_ERROR("No more turn");
        return;
    }

    switch (*nextTurn)
    {
    case 0:
        goStraght.reset(new GoStraght(msg->k, msg->distance + distance_after_go_straight));
        transferStateTo(NavigateState::GoingStraght);
        nextTurn++;
        break;
    case 1:
    case -1:
        goStraght.reset(new GoStraght(msg->k, msg->distance - distance_when_start_turn));
        transferStateTo(NavigateState::GoingStraghtBeforeTurn);
        break;
    case 2:
        goStraght.reset(new GoStraght(msg->k, msg->distance + distance_after_finish));
        transferStateTo(NavigateState::GoingStraght);
        nextTurn++;
        break;
    default:
        ROS_BREAK();
    }
}

void CommandComputer::transferStateTo(NavigateState state)
{
    switch (state)
    {
    case NavigateState::FollowingLine:
        currentNavigator = followLine.get();
        break;
    case NavigateState::GoingStraght:
    case NavigateState::GoingStraghtBeforeTurn:
    case NavigateState::GoingStraghtBeforeFinish:
        currentNavigator = goStraght.get();
        break;
    case NavigateState::Turning:
        currentNavigator = turn.get();
        break;
    case NavigateState::Finished:
        break;
    default:
        ROS_BREAK();
    }
    navigateState = state;
}
