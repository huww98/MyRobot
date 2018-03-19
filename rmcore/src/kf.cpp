#include "kf.h"

using namespace std;
using namespace std::chrono;

class InitialStep : public KalmanFilter::StepType
{
  private:
    using Base = KalmanFilter::StepType;

  public:
    InitialStep():Base(Base::TimePointType::min())
    {
        this->finishedState = RobotState();
    }

    virtual const StateType &Run(const StateType &initialState) override final {};
};

KalmanFilter::KalmanFilter(double baseWidth, RosDiffrentalController &controller)
    : controller(&controller), Base(make_unique<InitialStep>())
{
    InitEncoderUpdater(baseWidth);
}

void KalmanFilter::Predict(const ControlParameters &parameters)
{
    Base::Predict(parameters.time, make_shared<Predictor>(parameters.command, *controller, parameters.noise));
}

template<int idx>
void KalmanFilter::UpdateEncoder(const encoder::Data &data)
{
    Base::Update<idx>(data.time, make_unique<EncoderUpdater<idx>>(data)); //Todo: dropHistory
}

void KalmanFilter::UpdateImu(const imu::Data &data)
{
    Base::Update<2>(data.time, make_unique<ImuUpdater>(data));
}

Predictor::Predictor(const ControlCommand &cmd, const RosDiffrentalController &controller, const ControlNoise &noise)
    : cmd(cmd), controller(controller), noise(noise)
{
}

auto Predictor::GetParameters(const StateType &initialState, DurationType duration) -> PredictParameters
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

void InitEncoderUpdater(double baseWidth)
{
    EncoderUpdater<0>::H << 1, -baseWidth / 2, 0, 0, 0;
    EncoderUpdater<1>::H << 1, baseWidth / 2, 0, 0, 0;
}

#ifndef __INTELLISENSE__ //workaround https://github.com/Microsoft/vscode-cpptools/issues/871
template class EncoderUpdater<0>;
template class EncoderUpdater<1>;
#endif

ImuUpdater::UpdateParameters::HType ImuUpdater::H = (UpdateParameters::HType() << 0, 1, 0, 0, 0).finished();

ImuUpdater::ImuUpdater(const imu::Data &data)
{
    this->observationVector << data.angularVecocity;
    this->pH = &H;
    this->noiseCov << data.var;
}
