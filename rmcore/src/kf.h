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

struct ControlNoise
{
    double linear;
    double angular;
};

struct ControlParameters;

class KalmanFilter : public kf::KalmanFilter<5, 4, RobotState>
{
  private:
    using Base = kf::KalmanFilter<5, 4, RobotState>;
    RosDiffrentalController *controller;
    template <int idx>
    void UpdateEncoder(const encoder::Data &data);

  public:
    KalmanFilter(double baseWidth, RosDiffrentalController &controller);
    void Predict(const ControlParameters &parameters);
    void UpdateLeftEncoder(const encoder::Data &data) { UpdateEncoder<0>(data); }
    void UpdateRightEncoder(const encoder::Data &data) { UpdateEncoder<1>(data); }
    void UpdateImu(const imu::Data &data);
};

struct ControlParameters
{
    ControlCommand command;
    ControlNoise noise;
    KalmanFilter::TimePointType time;
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
    virtual PredictParameters GetParameters(const StateType &initialState, DurationType duration) override;
};

void InitEncoderUpdater(double baseWidth);

template <int idx>
class EncoderUpdater : public kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>
{
  private:
    using Base = kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>;
    static UpdateParameters::HType H;

  public:
    EncoderUpdater(const encoder::Data &data);
    friend void InitEncoderUpdater(double baseWidth);
};

using LeftEncoderUpdater = EncoderUpdater<0>;
using RightEncoderUpdater = EncoderUpdater<1>;

class ImuUpdater : public kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>
{
  private:
    using Base = kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>;
    static UpdateParameters::HType H;

  public:
    ImuUpdater(const imu::Data &data);
};

#endif
