#include "kf/kalman_filter.h"

struct State
{
    double LeftVelocity;
    double RightVelocity;
};

class KalmanFilter : public kf::KalmanFilter<2>
{
  private:
    using Base = kf::KalmanFilter<2>;

  public:
    State GetLatestState() const;
};
