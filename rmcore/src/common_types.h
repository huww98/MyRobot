#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include "kf/state.h"

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

    auto V() { return this->State.segment<2>(0); }
    auto V() const { return this->State.segment<2>(0); }
};

struct AccelerationCommand
{
    Eigen::Vector2d vec;

    double &linear() {return vec(0);}
    double linear() const {return vec(0);}

    double &angular() {return vec(1);}
    double angular() const {return vec(1);}
};

#endif
