#include "kf.h"

using namespace std;
using namespace std::chrono;

class InitialStep : public KalmanFilter::StepType
{
  private:
    using Base = KalmanFilter::StepType;

  public:
    InitialStep(const RobotState &initState):Base(Base::TimePointType::min())
    {
        this->finishedState = initState;
    }

    virtual const StateType &Run(const StateType &initialState) override final {};
};

KalmanFilter::KalmanFilter(double baseWidth, RosDiffrentalController &controller, const RobotState &initState)
    : controller(&controller), Base(make_unique<InitialStep>(initState))
{
    InitEncoderUpdater(baseWidth);
}

void KalmanFilter::Predict(const ControlParameters &parameters)
{
    Base::Predict(parameters.time, make_shared<Predictor>(parameters.command, *controller));
}

constexpr int encoderUpdateLineOffset = 0, imuUpdateLine = 2;

template<int idx>
void KalmanFilter::UpdateEncoder(const encoder::Data &data)
{
    Base::Update<encoderUpdateLineOffset + idx>(data.time, make_unique<EncoderUpdater<idx>>(data));
}

template void KalmanFilter::UpdateEncoder<0>(const encoder::Data &data);
template void KalmanFilter::UpdateEncoder<1>(const encoder::Data &data);

void KalmanFilter::UpdateImu(const imu::Data &data)
{
    Base::Update<imuUpdateLine>(data.time, make_unique<ImuUpdater>(data));
}

Predictor::Predictor(const ControlVoltage &cmd, const RosDiffrentalController &controller)
    : cmd(cmd), controller(controller)
{
}

auto Predictor::GetParameters(const StateType &initialState, DurationType duration) -> PredictParameters
{
    double t = duration_cast<chrono::duration<double>>(duration).count();

    auto predictedA = controller.PredictAcceleration(initialState, cmd);
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
    Eigen::Vector2d dirVecDerivative(-sin(directionAngle), cos(directionAngle));
    Eigen::Vector2d newPos = initialState.Position() + dirVec * dist;
    params.NextStateVec << v, omega, newPos, theta;
    double halfTSquare = 0.5 * t * t;
    params.F << Eigen::Matrix2d::Identity() + predictedA.jacobianOfVelocity * t, Eigen::Matrix<double, 2, 3>::Zero(),
        (t + halfTSquare * predictedA.jacobianOfVelocity(0, 0)) * dirVec, dist * dirVecDerivative * t / 2 + halfTSquare * predictedA.jacobianOfVelocity(0, 1) * dirVec, Eigen::Matrix2d::Identity(), dist * dirVecDerivative,
        halfTSquare * predictedA.jacobianOfVelocity(1, 0), t + halfTSquare * predictedA.jacobianOfVelocity(1, 1), 0, 0, 1;

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
    EncoderUpdater<0>::H << 1, -baseWidth / 2, 0, 0, 0;
    EncoderUpdater<1>::H << 1, baseWidth / 2, 0, 0, 0;
}

template class EncoderUpdater<0>;
template class EncoderUpdater<1>;

ImuUpdater::UpdateParameters::HType ImuUpdater::H = (UpdateParameters::HType() << 0, 1, 0, 0, 0).finished();

ImuUpdater::ImuUpdater(const imu::Data &data)
{
    this->observationVector << data.angularVecocity;
    this->pH = &H;
    this->noiseCov << data.var;
}
