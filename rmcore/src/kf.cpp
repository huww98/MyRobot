#include "kf.h"
#include "kf_step.h"

using namespace std;

KalmanFilter::StepPtr buildFirstStep()
{
    KalmanFilter::StepPtr firstStep(new PredictStep(PredictStep::TimePoint::min(), ControlCommand{{0, 0}, {0, 0}}));
    firstStep->Run(PredictStep::StateType{PredictStep::StateType::ValueVector::Zero(), PredictStep::StateType::CovMat::Zero()});
    return firstStep;
}

KalmanFilter::KalmanFilter() : Base(buildFirstStep())
{
}

State KalmanFilter::GetLatestState() const
{
    State state;
    auto baseState = Base::GetLatestState();
    state.LeftVelocity = baseState.State(0);
    state.RightVelocity = baseState.State(1);
}
