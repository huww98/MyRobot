#include "kf/step.h"
#include "ros_encoder_data.h"
#include "ros_imu_data.h"

struct MotorCommand
{
    double acceleration;
    double variance;
};

struct ControlCommand
{
    MotorCommand left;
    MotorCommand right;
};

class PredictStep : public kf::PredictStep<2, 2>
{
  private:
    using Base = kf::PredictStep<2, 2>;
    Eigen::Matrix2d covMatBase;

  public:
    PredictStep(TimePoint time, const ControlCommand &cmd);
    void GenerateParameters(Duration duration) override;
    Base *Clone() const override;
};

template<int idx>
class EncoderUpdateStep : public kf::UpdateStep<2, 1>
{
  public:
    EncoderUpdateStep(const encoder::Data &data);

  private:
    using Base = kf::UpdateStep<2, 1>;
};

using LeftEncoderUpdateStep = EncoderUpdateStep<0>;
using RightEncoderUpdateStep = EncoderUpdateStep<1>;

class GyroUpdateStep : public kf::UpdateStep<2, 1>
{
  public:
    GyroUpdateStep(const imu::Data &data);

    static Eigen::Matrix<double, 1, 2> h;

  private:
    using Base = kf::UpdateStep<2, 1>;
};
