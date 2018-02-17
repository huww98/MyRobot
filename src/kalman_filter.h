#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

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
    SystemModel<stateCount, ControlVectorLength> SystemModel;
    Eigen::Matrix<double, ControlVectorLength, 1> ControlVector;
};

template <int stateCount, int ObservationVectorLength>
struct ObservationModel
{
    Eigen::Matrix<double, stateCount, ObservationVectorLength> H;
    Eigen::Matrix<double, ObservationVectorLength, ObservationVectorLength> NoiseCov;
};

template <int stateCount, int ObservationVectorLength>
struct UpdateParameters
{
    ObservationModel<stateCount, ObservationVectorLength> ObservationModel;
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
    virtual void SetDuration(Duration duration) {}

  private:
    TimePoint time;
    StateType finishedState;
};

template <int stateCount, int ControlVectorLength>
class PredictStep : public Step<stateCount>
{
  public:
    using ControlVector = Eigen::Matrix<double, ControlVectorLength, 1>;
    using PredictParametersType = PredictParameters<stateCount, ControlVectorLength>;

    PredictStep(TimePoint time, const PredictParametersType &params) : Step(time), predictParameters(params) {}

    virtual const StateType &Run(const StateType &initialState) override
    {
        auto &F = predictParameters.SystemModel.F;
        auto &B = predictParameters.SystemModel.B;

        finishedState.State = F * initialState.State + B * predictParameters.ControlVector;
        finishedState.Covariance = F * initialState.Covariance * F.transpose() + predictParameters.SystemModel.NoiseCov;
        return finishedState;
    }

  private:
    PredictParametersType predictParameters;
};

template <int stateCount, int ControlVectorLength, int ObservationVectorLength>
class UpdateStep : public PredictStep<stateCount, ControlVectorLength>
{
  public:
    using ObservationVector = Eigen::Matrix<double, ObservationVectorLength, 1>;
    using ObservationCovMat = Eigen::Matrix<double, ObservationVectorLength, ObservationVectorLength>;
    using UpdateParametersType = UpdateParameters<stateCount, ObservationVectorLength>;

    UpdateStep(TimePoint time, const PredictParametersType &pParams, const UpdateParametersType &uParams)
        : PredictStep(time, pParams), updateParameters(uParams) {}

    virtual const StateType &Run(const StateType &initialState) override
    {
        auto &H = updateParameters.ObservationModel.H;
        auto &ObservationNoiseCov = updateParameters.ObservationModel.NoiseCov;
        auto predictedState = PredictStep::Run(initialState);

        ObservationVector innovation = updateParameters.ObservationVector - H * predictedState.State;
        ObservationCovMat innovationCov = ObservationNoiseCov + H * predictedState.Covariance * H.transpose();
        Eigen::Matrix<double, stateCount, ObservationVectorLength> gain = predictedState.Covariance * H.transpose() * innovationCov.inverse();
        finishedState.State = predictedState.State + gain * innovation;
        StateCovMat temp = StateCovMat::Identity() - gain * H;
        finishedState.Covariance = temp * predictedState.Covariance * temp.transpose() + gain * ObservationNoiseCov * gain.transpose();
    }

  private:
    UpdateParametersType updateParameters;
};

template <int stateCount>
class KalmanFilter
{
  public:
    using StepType = Step<stateCount>;

    void insertStep(std::unique_ptr<StepType>&& newStep)
    {
        auto insertPos = pendingSteps.end();
        while (insertPos->GetTime() > newStep->GetTime())
            insertPos--;
        auto state = insertPos->GetFinishedState();
        auto lastUpdateTime = insertPos->GetTime();
        insertPos++;
        auto runningPos = pendingSteps.insert(insertPos, newStep);

        newStep->SetDuration(newStep->GetTime() - lastUpdateTime);
        insertPos->SetDuration(insertPos->GetTime() - newStep->GetTime());

        for (; runningPos != pendingSteps.end(); runningPos++)
            state = runningPos->Run(state);
    }
    const State<stateCount> &GetLatestState() const { return (--pendingSteps.end())->GetFinishedState(); }

  private:
    std::list<std::unique_ptr<StepType>> pendingSteps;
};
}

#endif
