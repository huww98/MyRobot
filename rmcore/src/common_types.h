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
};

struct AccelerationCommand
{
    double linear;
    double angular;
};

#endif
