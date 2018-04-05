#ifndef KF_H
#define KF_H

#include "kf/kalman_filter.h"
#include "ros_encoder_data.h"
#include "ros_imu_data.h"
#include "ros_controller.h"
#include "common_types.h"

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
    KalmanFilter(double baseWidth, RosDiffrentalController &controller, const RobotState &initState);
    void Predict(const ControlParameters &parameters);
    void UpdateLeftEncoder(const encoder::Data &data) { UpdateEncoder<0>(data); }
    void UpdateRightEncoder(const encoder::Data &data) { UpdateEncoder<1>(data); }
    void UpdateImu(const imu::Data &data);
};

struct ControlParameters
{
    ControlVoltage command;
    ControlNoise noise;
    KalmanFilter::TimePointType time;
};

class Predictor : public KalmanFilter::PredictorType
{
  private:
    using Base = KalmanFilter::PredictorType;
    ControlVoltage cmd;
    ControlNoise noise;
    DurationType duration;
    RosDiffrentalController controller;

  public:
    Predictor(const ControlVoltage &cmd, const RosDiffrentalController &controller, const ControlNoise &noise);
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
