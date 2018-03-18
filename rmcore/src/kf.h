#ifndef KF_H
#define KF_H

#include "kf/kalman_filter.h"

class RobotState : public kf::State<5>
{
  public:
    double &Velocity() { return this->State(0); }
    double Velocity() const { return this->State(0); }

    double &AngularVelocity() { return this->State(1); };
    double AngularVelocity() const { return this->State(1); };

    double &Angle() { return this->State(4); }
    double Angle() const { return this->State(4); }

    auto Position() { return this->State.segment<2>(2); }
    auto Position() const { return this->State.segment<2>(2); }
};

class KalmanFilter : public kf::KalmanFilter<5, RobotState>
{
  private:
    using Base = kf::KalmanFilter<5, RobotState>;

  public:
    KalmanFilter();
};

#endif
