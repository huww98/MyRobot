#ifndef KF_STEP_H
#define KF_STEP_H

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <memory>

namespace kf
{
template <int stateCount, int ControlVectorLength>
struct SystemModel
{
    Eigen::Matrix<double, stateCount, stateCount> F;
    Eigen::Matrix<double, stateCount, ControlVectorLength> B;
    Eigen::Matrix<double, stateCount, stateCount> NoiseCov;
};

template <int stateCount, int ControlVectorLength>
struct PredictParameters
{
    SystemModel<stateCount, ControlVectorLength> Model;
    Eigen::Matrix<double, ControlVectorLength, 1> ControlVector;
};

template <int stateCount, int ObservationVectorLength>
struct ObservationModel
{
    Eigen::Matrix<double, ObservationVectorLength, stateCount> H;
    Eigen::Matrix<double, ObservationVectorLength, ObservationVectorLength> NoiseCov;
};

template <int stateCount, int ObservationVectorLength>
struct UpdateParameters
{
    ObservationModel<stateCount, ObservationVectorLength> Model;
    Eigen::Matrix<double, ObservationVectorLength, 1> ObservationVector;
};

template <int stateCount>
struct State
{
    using ValueVector = Eigen::Matrix<double, stateCount, 1>;
    using CovMat = Eigen::Matrix<double, stateCount, stateCount>;
    ValueVector State;
    CovMat Covariance;
};

template <int stateCount>
class Step
{
  public:
    using StateType = State<stateCount>;
    using TimePoint = std::chrono::steady_clock::time_point;
    using Duration = std::chrono::steady_clock::duration;

    Step(TimePoint time) : time(time) {}

    const StateType &GetFinishedState() const { return finishedState; }
    TimePoint GetTime() const { return time; }

    virtual const StateType &Run(const StateType &initialState) = 0;
    virtual void GenerateParameters(Duration duration) {}

  protected:
    TimePoint time;
    StateType finishedState;
};

template <int stateCount>
class PredictStepBase : public Step<stateCount>{
  private:
    using Base = Step<stateCount>;

  public:
    using Base::Base;
    virtual PredictStepBase *Clone() const = 0;
};

template <int stateCount, int ControlVectorLength>
class PredictStep : public PredictStepBase<stateCount>
{
  private:
    using Base = PredictStepBase<stateCount>;

  public:
    using typename Base::StateType;
    using typename Base::TimePoint;
    using ControlVector = Eigen::Matrix<double, ControlVectorLength, 1>;
    using PredictParametersType = PredictParameters<stateCount, ControlVectorLength>;

    PredictStep(TimePoint time) : Base(time) {}

    virtual const StateType &Run(const StateType &initialState) override
    {
        auto &F = predictParameters.Model.F;
        auto &B = predictParameters.Model.B;

        this->finishedState.State = F * initialState.State + B * predictParameters.ControlVector;
        this->finishedState.Covariance = F * initialState.Covariance * F.transpose() + predictParameters.Model.NoiseCov;
        return this->finishedState;
    }

  protected:
    PredictParametersType predictParameters;
};

template <int stateCount>
class UpdateStepBase : public Step<stateCount>
{
  private:
    using Base = Step<stateCount>;

  public:
    using Base::Base;
    using PredictStepType = PredictStepBase<stateCount>;
    using PredictStepPtr = std::unique_ptr<PredictStepType>;

    void SetPredictStep(PredictStepPtr &&pStep)
    {
        predictStep = std::move(pStep);
    }

    PredictStepType *GetPredictStep()
    {
        return predictStep.get();
    }

  protected:
    PredictStepPtr predictStep;
};

template <int stateCount, int ObservationVectorLength>
class UpdateStep : public UpdateStepBase<stateCount>
{
  private:
    using Base = UpdateStepBase<stateCount>;

  public:
    using typename Base::StateType;
    using typename Base::TimePoint;
    using typename Base::Duration;
    using ObservationVector = Eigen::Matrix<double, ObservationVectorLength, 1>;
    using ObservationCovMat = Eigen::Matrix<double, ObservationVectorLength, ObservationVectorLength>;
    using UpdateParametersType = UpdateParameters<stateCount, ObservationVectorLength>;

    UpdateStep(TimePoint time)
        : Base(time)
    {}

    virtual const StateType &Run(const StateType &initialState) override
    {
        auto &H = updateParameters.Model.H;
        auto &ObservationNoiseCov = updateParameters.Model.NoiseCov;
        auto &predictedState = this->predictStep->Run(initialState);

        ObservationVector innovation = updateParameters.ObservationVector - H * predictedState.State;
        ObservationCovMat innovationCov = ObservationNoiseCov + H * predictedState.Covariance * H.transpose();
        Eigen::Matrix<double, stateCount, ObservationVectorLength> gain = predictedState.Covariance * H.transpose() * innovationCov.inverse();
        this->finishedState.State = predictedState.State + gain * innovation;
        typename StateType::CovMat temp = StateType::CovMat::Identity() - gain * H;
        this->finishedState.Covariance = temp * predictedState.Covariance * temp.transpose() + gain * ObservationNoiseCov * gain.transpose();
    }

    virtual void GenerateParameters(Duration duration) override
    {
        this->predictStep->GenerateParameters(duration);
    }

  protected:
    UpdateParametersType updateParameters;
};
}

#endif
