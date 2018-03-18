#include "kf_step.h"

using namespace std;
using namespace std::chrono;

Predictor::Predictor(const ControlCommand &cmd, const RosDiffrentalController &controller, const ControlNoise &noise)
    : cmd(cmd), controller(controller), noise(noise)
{
}

void Predictor::SetDuration(DurationType dur)
{
    this->duration = dur;
}

auto Predictor::GetParameters(const StateType &initialState) -> PredictParameters
{
    double t = duration_cast<chrono::duration<double>>(duration).count();

    auto predictedA = controller.PredictAcceleration(initialState.Velocity(), initialState.AngularVelocity(), cmd);
    PredictParameters params;
    double v = initialState.Velocity() + predictedA.linear * t;
    double omega = initialState.AngularVelocity() + predictedA.angular * t;
    double meanV = (initialState.Velocity() + v) / 2;
    double meanOmega = (initialState.AngularVelocity() + omega) / 2;
    double deltaTheta = meanOmega * t;
    double theta = initialState.Angle() + deltaTheta;
    double dist = meanV * t; //approximate
    double directionAngle = initialState.Angle() + deltaTheta / 2;
    Eigen::Vector2d dirVec(cos(directionAngle), sin(directionAngle));
    Eigen::Vector2d dirVecDerivativeToTheta(-sin(directionAngle), cos(directionAngle));
    Eigen::Vector2d newPos = initialState.Position() + dirVec * dist;
    params.NextStateVec << v, omega, newPos, theta;
    params.F << Eigen::Matrix2d::Identity() + predictedA.jacobianOfVelocity * t, Eigen::Matrix<double, 2, 3>::Zero(),
        t * dirVec, dist * dirVecDerivativeToTheta * t, Eigen::Matrix2d::Identity(), dist * dirVecDerivativeToTheta,
        0, t, 0, 0, 1;

    // 噪音计算是一个近似。方差本应该和时间的平方成正比，但在这个实例中，本次控制与之前的控制结果关系较大，
    // 其协方差与时间成正比，且方差和写方差相比应该可以忽略不计。这样计算误差，在时间间隔较小时可使速度的
    // 方差保持在一个值。既不会无限增大，也不会趋近于0.
    params.NoiseCov.setZero();
    params.NoiseCov(0, 0) = noise.linear * t;
    params.NoiseCov(1, 1) = noise.angular * t;

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

template <>
void EncoderUpdater<0>::Init(double baseWidth)
{
    H << 1, -baseWidth / 2, 0, 0, 0;
}

template <>
void EncoderUpdater<1>::Init(double baseWidth)
{
    H << 1, baseWidth / 2, 0, 0, 0;
}

#ifndef __INTELLISENSE__ //workaround https://github.com/Microsoft/vscode-cpptools/issues/871
template class EncoderUpdateStep<0>;
template class EncoderUpdateStep<1>;
#endif

GyroUpdater::UpdateParameters::HType GyroUpdater::H = (UpdateParameters::HType() << 0, 1, 0, 0, 0).finished();

GyroUpdater::GyroUpdater(const imu::Data &data)
{
    this->observationVector << data.angularVecocity;
    this->pH = &H;
    this->noiseCov << data.var;
}
