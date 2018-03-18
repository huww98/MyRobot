#include "kf.h"
#include "kf_step.h"

using namespace std;

class InitialStep : public KalmanFilter::StepType
{
  private:
    using Base = KalmanFilter::StepType;

  public:
    InitialStep():Base(Base::TimePointType::min())
    {
        this->finishedState = RobotState();
    }

    virtual const StateType &Run(const StateType &initialState) override final {};
};

KalmanFilter::KalmanFilter() : Base(make_unique<InitialStep>())
{
}
