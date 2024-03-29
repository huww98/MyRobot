#include "kf.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

KalmanFilter::KalmanFilter(double baseWidth, RosDiffrentalController &controller, const RobotState &initState)
    : Base(initState, steady_clock::now(), PredictorPtr(new Predictor(ControlVoltage{0,0}, controller))), controller(&controller)
{
    InitEncoderUpdater(baseWidth);
}

void KalmanFilter::Predict(const ControlParameters &parameters)
{
    Base::Predict(parameters.time, PredictorPtr(new Predictor(parameters.command, *controller)));
}

constexpr int encoderUpdateLineOffset = 0, imuUpdateLine = 2;

template<int idx>
void KalmanFilter::UpdateEncoder(const encoder::Data &data)
{
    Base::Update<encoderUpdateLineOffset + idx>(data.time, Base::UpdaterPtr(new EncoderUpdater<idx>(data)));
}

template void KalmanFilter::UpdateEncoder<0>(const encoder::Data &data);
template void KalmanFilter::UpdateEncoder<1>(const encoder::Data &data);

void KalmanFilter::UpdateImu(const imu::Data &data)
{
    Base::Update<imuUpdateLine>(data.time, Base::UpdaterPtr(new ImuUpdater(data)));
}

Predictor::Predictor(const ControlVoltage &cmd, const RosDiffrentalController &controller)
    : cmd(cmd), controller(controller)
{
}

auto Predictor::GetParameters(const StateType &initialState, DurationType duration) -> PredictParameters
{
    double t = duration_cast<chrono::duration<double>>(duration).count();
    ROS_ASSERT(t >= 0.0);

    auto predictedA = controller.PredictAcceleration(initialState, cmd);
    PredictParameters params;
    Vector2d a = predictedA.accel.vec;
    Vector2d v = initialState.V() + a * t;
    double halfTSquare = 0.5 * t * t;
    Vector2d x = initialState.X() + initialState.V() * t + halfTSquare * a;
    params.NextStateVec << v, x;
    params.F.block<2,2>(0,0) = Eigen::Matrix2d::Identity() + predictedA.jacobianOfVelocity * t;
    params.F.block<2,2>(2,2) = Eigen::Matrix2d::Identity();
    params.F.block<2,2>(2,0) = t * Eigen::Matrix2d::Identity() + halfTSquare * predictedA.jacobianOfVelocity;
    params.F.block<2,2>(0,2).setZero();

    // 噪音计算是一个近似。方差本应该和时间的平方成正比，但在这个实例中，本次控制与之前的控制结果关系较大，
    // 其协方差与时间成正比，且方差和协方差相比应该可以忽略不计。这样计算误差，在时间间隔较小时可使速度的
    // 方差保持在一个值。既不会无限增大，也不会趋近于0.
    params.NoiseCov.setZero();
    params.NoiseCov.block<2,2>(0,0) = predictedA.Covariance * t;
    return params;
}

template <int idx>
EncoderUpdater<idx>::UpdateParameters::HType EncoderUpdater<idx>::H = UpdateParameters::HType::Zero();

template <int idx>
EncoderUpdater<idx>::EncoderUpdater(const encoder::Data &data)
{
    this->pH = &H;
    this->observationVector << data.velocity;
    this->noiseCov << data.var;
}

void InitEncoderUpdater(double baseWidth)
{
    EncoderUpdater<0>::H << 1, -baseWidth / 2, 0, 0;
    EncoderUpdater<1>::H << 1, baseWidth / 2, 0, 0;
}

template class EncoderUpdater<0>;
template class EncoderUpdater<1>;

ImuUpdater::UpdateParameters::HType ImuUpdater::H = (UpdateParameters::HType() << 0, 1, 0, 0).finished();

ImuUpdater::ImuUpdater(const imu::Data &data)
{
    this->observationVector << data.angularVecocity;
    this->pH = &H;
    this->noiseCov << data.var;
}
