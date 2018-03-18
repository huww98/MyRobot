#include "kf/step.h"
#include "kf.h"
#include "ros_encoder_data.h"
#include "ros_imu_data.h"
#include "ros_controller.h"

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
  private:
    using Base = kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>;
    static UpdateParameters::HType H;

  public:
    EncoderUpdater(const encoder::Data &data);
    static void Init(double baseWidth);
};

using LeftEncoderUpdater = EncoderUpdater<0>;
using RightEncoderUpdater = EncoderUpdater<1>;

class GyroUpdater : public kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>
{
  private:
    using Base = kf::LinearUpdater<KalmanFilter::StateCount, 1, KalmanFilter::StateType>;
    static UpdateParameters::HType H;

  public:
    GyroUpdater(const imu::Data &data);
};
