#ifndef KF_STEP_H
#define KF_STEP_H

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <memory>

#include "state.h"

namespace kf
{
template <int stateCount, typename StateT = State<stateCount>>
class Step
{
  public:
    using TimePointType = std::chrono::steady_clock::time_point;
    using DurationType = std::chrono::steady_clock::duration;
    using StateType = StateT;
    static_assert(std::is_base_of<State<stateCount>, StateType>::value, "StateType must be a descendant of State<stateCount>");

    Step(TimePointType time) : time(time) {}

    const StateType &GetFinishedState() const { return finishedState; }
    TimePointType GetTime() const { return time; }

    virtual const StateType &Run(const StateType &initialState) = 0;
    void SetDuration(DurationType d) { this->duration = d; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    DurationType duration;
    TimePointType time;
    StateType finishedState;
};

template <int stateCount, typename StateT = State<stateCount>>
class InitialStep : public Step<stateCount, StateT>
{
  private:
    using Base = Step<stateCount, StateT>;

  public:
    InitialStep(const StateT &initState):Base(Base::TimePointType::min())
    {
        this->finishedState = initState;
    }

    virtual const StateT &Run(const StateT &initialState) override final { return this->finishedState; };
};

template <int stateCount, typename StateT = State<stateCount>>
class Predictor
{
  public:
    using StateType = StateT;
    using DurationType = std::chrono::steady_clock::duration;

    struct PredictParameters
    {
        using FType = Eigen::Matrix<double, stateCount, stateCount>;
        using NoiseCovType = Eigen::Matrix<double, stateCount, stateCount>;
        using StateVecType = typename StateType::ValueVector;

        StateVecType NextStateVec;
        FType F;
        NoiseCovType NoiseCov;
    };

    virtual StateType Predict(const StateType &initialState, DurationType duration)
    {
        auto params = this->GetParameters(initialState, duration);

        StateType nextState;
        nextState.State = params.NextStateVec;
        nextState.Covariance = params.F * initialState.Covariance * params.F.transpose() + params.NoiseCov;
        return nextState;
    }

    virtual PredictParameters GetParameters(const StateType &initialState, DurationType duration) = 0;
};

template <int stateCount, typename StateType = State<stateCount>>
class PredictStep : public Step<stateCount, StateType>
{
  private:
    using Base = Step<stateCount, StateType>;

  public:
    using PredictorType = Predictor<stateCount, StateType>;
    using PredictorPtr = std::shared_ptr<PredictorType>;
    using typename Base::DurationType;
    using typename Base::TimePointType;

    PredictStep(TimePointType time, PredictorPtr &predictor)
        : Base(time), predictor(predictor)
    {
    }

    virtual const StateType &Run(const StateType &initialState) override
    {
        this->finishedState = predictor->Predict(initialState, this->duration);
        return this->finishedState;
    }

    PredictorPtr GetPredictor() { return this->predictor; }

  private:
    PredictorPtr predictor;
};

template <int stateCount, typename StateType = State<stateCount>>
class UpdaterBase
{
  public:
    virtual StateType Update(const StateType &predictedState) = 0;
};

template <int stateCount, int ObservationVectorLength, typename StateType = State<stateCount>>
class Updater : public UpdaterBase<stateCount, StateType>
{
  public:
    struct UpdateParameters
    {
        using HType = Eigen::Matrix<double, ObservationVectorLength, stateCount>;
        using NoiseCovType = Eigen::Matrix<double, ObservationVectorLength, ObservationVectorLength>;
        using ObservationVectorType = Eigen::Matrix<double, ObservationVectorLength, 1>;

        ObservationVectorType Innovation;
        HType H;
        NoiseCovType NoiseCov;
    };

    virtual UpdateParameters GetParameters(const StateType &predictedState) = 0;

    virtual StateType Update(const StateType &predictedState) override
    {
        auto params = GetParameters(predictedState);
        auto &noiseCov = params.NoiseCov;
        auto &H = params.H;
        auto &innovation = params.Innovation;
        StateType nextState;

        typename UpdateParameters::NoiseCovType innovationCov = noiseCov + H * predictedState.Covariance * H.transpose();
        Eigen::Matrix<double, stateCount, ObservationVectorLength> gain = predictedState.Covariance * H.transpose() * innovationCov.inverse();
        nextState.State = predictedState.State + gain * innovation;
        nextState.Covariance = StateType::CovMat::Identity() - gain * H * predictedState.Covariance;
        return nextState;
    }
};

template <int stateCount, int ObservationVectorLength, typename StateType = State<stateCount>>
class LinearUpdater : public Updater<stateCount, ObservationVectorLength, StateType>
{
  private:
    using Base = Updater<stateCount, ObservationVectorLength, StateType>;

  public:
    using typename Base::UpdateParameters;

  protected:
    typename UpdateParameters::HType *pH;
    typename UpdateParameters::NoiseCovType noiseCov;
    typename UpdateParameters::ObservationVectorType observationVector;

  public:
    virtual UpdateParameters GetParameters(const StateType &predictedState) override
    {
        UpdateParameters params;
        auto &H = *pH;
        params.Innovation = observationVector - H * predictedState.State;
        params.H = H;
        params.NoiseCov = noiseCov;
        return params;
    }
};

template <int stateCount, typename StateType = State<stateCount>>
class UpdateStep : public Step<stateCount, StateType>
{
  private:
    using Base = Step<stateCount, StateType>;

  public:
    using typename Base::DurationType;
    using typename Base::TimePointType;
    using PredictorType = Predictor<stateCount, StateType>;
    using PredictorPtr = std::shared_ptr<PredictorType>;
    using UpdaterType = UpdaterBase<stateCount, StateType>;
    using UpdaterPtr = std::unique_ptr<UpdaterType>;

    UpdateStep(TimePointType time, PredictorPtr &predictor, UpdaterPtr &&updater)
        : Base(time), predictor(predictor), updater(std::move(updater))
    {
    }

    virtual const StateType &Run(const StateType &initialState) override
    {
        this->finishedState = this->updater->Update(this->predictor->Predict(initialState, this->duration));
        return this->finishedState;
    }

    PredictorPtr GetPredictor() { return this->predictor; }
    void ReplacePredictor(PredictorPtr &p) { predictor = p; }

  protected:
    PredictorPtr predictor;
    UpdaterPtr updater;
};
}

#endif
