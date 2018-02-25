#include "kf.h"

State KalmanFilter::GetLatestState() const
{
    State state;
    auto baseState = Base::GetLatestState();
    state.LeftVelocity = baseState.State(0);
    state.RightVelocity = baseState.State(1);
}
