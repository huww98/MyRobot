#include "kf_step.h"

using namespace std;
using namespace std::chrono;

PredictStep::PredictStep(TimePoint time, const ControlCommand &cmd)
    : Base(time)
{
    this->predictParameters.ControlVector << cmd.left.acceleration, cmd.right.acceleration;
    this->predictParameters.Model.B << 1.0, 1.0;
    covMatBase << cmd.left.variance, 0,
                  0                , cmd.right.variance;
}

PredictStep::Base *PredictStep::Clone() const
{
    return new PredictStep(*this);
}

void PredictStep::GenerateParameters(Duration dur)
{
    double secs = duration_cast<duration<double>>(dur).count();
    this->predictParameters.Model.F << secs, secs;
    this->predictParameters.Model.NoiseCov = covMatBase * secs;
}

template <int idx>
EncoderUpdateStep<idx>::EncoderUpdateStep(const encoder::Data &data)
    : Base(data.time)
{
    this->updateParameters.ObservationVector << data.velocity;
    this->updateParameters.Model.H.setZero();
    this->updateParameters.Model.H(idx) = 1;
    this->updateParameters.Model.NoiseCov << data.var;
}

#ifndef __INTELLISENSE__ //workaround https://github.com/Microsoft/vscode-cpptools/issues/871
template class EncoderUpdateStep<0>;
template class EncoderUpdateStep<1>;
#endif

Eigen::Matrix<double, 1, 2> GyroUpdateStep::h;

GyroUpdateStep::GyroUpdateStep(const imu::Data &data)
    : Base(data.time)
{
    this->updateParameters.ObservationVector << data.angularVecocity;
    this->updateParameters.Model.H = h;
    this->updateParameters.Model.NoiseCov << data.var;
}
