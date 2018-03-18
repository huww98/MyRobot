#ifndef KF_H
#define KF_H

#include "kf/kalman_filter.h"
#include "ros_encoder_data.h"
#include "ros_imu_data.h"
#include "ros_controller.h"

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
    KalmanFilter(double baseWidth);
};

struct ControlNoise
{
    double linear;
    double angular;
};

class Predictor : public KalmanFilter::PredictorType
{
  private:
    using Base = KalmanFilter::PredictorType;
    ControlCommand cmd;
    ControlNoise noise;
    DurationType duration;
    RosDiffrentalController controller;

  public:
    Predictor(const ControlCommand &cmd, const RosDiffrentalController &controller, const ControlNoise &noise);
    void SetDuration(DurationType duration) override;
    virtual PredictParameters GetParameters(const StateType &initialState) override;
    virtual StateType Predict(const StateType &initialState) override;
};

template <int idx>
class EncoderUpdater : public kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>
{
    friend void InitEncoderUpdater(double baseWidth);

  private:
    using Base = kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>;
    static UpdateParameters::HType H;

  public:
    EncoderUpdater(const encoder::Data &data);
};

using LeftEncoderUpdater = EncoderUpdater<0>;
using RightEncoderUpdater = EncoderUpdater<1>;

void InitEncoderUpdater(double baseWidth);

class GyroUpdater : public kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>
{
  private:
    using Base = kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>;
    static UpdateParameters::HType H;

  public:
    GyroUpdater(const imu::Data &data);
};

#endif
