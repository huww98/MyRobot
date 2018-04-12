#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <memory>
#include <array>
#include <algorithm>
#include <functional>
#include <iostream>
#include "step.h"

namespace kf
{
template <int stateCount, int ParallelUpdateCount, typename StateT = State<stateCount>>
class KalmanFilter
{
  public:
    using StateType = StateT;
    using StepType = Step<stateCount, StateType>;
    using StepPtr = std::unique_ptr<StepType>;
    using PredictStepType = PredictStep<stateCount, StateType>;
    using UpdateStepType = UpdateStep<stateCount, StateType>;

    using TimePointType = typename StepType::TimePointType;

    using PredictorType = typename PredictStepType::PredictorType;
    using PredictorPtr = typename PredictStepType::PredictorPtr;

    using UpdaterType = typename UpdateStepType::UpdaterType;
    using UpdaterPtr = typename UpdateStepType::UpdaterPtr;

    template <int ObservationVectorLength>
    using Updater = Updater<stateCount, ObservationVectorLength, StateType>;

    using StepList = std::list<StepPtr, Eigen::aligned_allocator<StepPtr>>;

    static int constexpr StateCount = stateCount;

    KalmanFilter(const StateType &initState, PredictorPtr &&predictor)
    {
        pendingSteps.push_back(StepPtr(new InitialStep<stateCount, StateType>(initState)));
        lastUpdatePos.fill(pendingSteps.begin());
        invalidStateBegin = pendingSteps.end();
        Predict((*pendingSteps.rbegin())->GetTime() + typename TimePointType::duration(1), std::move(predictor));
        // Add 1 duration to make PredictStep go after InitialStep
    }

    void Predict(TimePointType time, PredictorPtr &&predictor)
    {
        auto beforePos = findBeforePos(time);
        auto afterPos = std::next(beforePos);
        for (auto updatePos = afterPos; updatePos != pendingSteps.end(); updatePos++)
        {
            if (auto uStep = dynamic_cast<UpdateStepType *>(updatePos->get()))
                uStep->ReplacePredictor(predictor);
            else
                break;
        }
        doInsert<0>(beforePos, StepPtr(new PredictStepType(time, predictor)));
    }

    template <int UpdateLine>
    void Update(TimePointType time, UpdaterPtr &&updater)
    {
        auto beforePos = findBeforePos(time);
        PredictorPtr predictor;

        if (auto pStep = dynamic_cast<PredictStepType *>(beforePos->get()))
        {
            predictor = pStep->GetPredictor();
        }
        else if (auto uStep = dynamic_cast<UpdateStepType *>(beforePos->get()))
        {
            predictor = uStep->GetPredictor();
        }
        else
        {
            assert(false);
        }

        doInsert<UpdateLine + 1>(beforePos, StepPtr(new UpdateStepType(time, predictor, std::move(updater))));
    }

    const StateType &GetLatestState()
    {
        UpdateToLatest();

        auto &lastestStep = *pendingSteps.rbegin();
        return lastestStep->GetFinishedState();
    }

    void SetPredictEndTime(TimePointType t)
    {
        predictEndTime = t;
        auto &lastStep = *pendingSteps.rbegin();
        lastStep->SetDuration(t - lastStep->GetTime());

        // last state become invalid
        if (invalidStateBegin == pendingSteps.end() && pendingSteps.size() > 1)
            invalidStateBegin--;
    }

    void UpdateToLatest()
    {
        UpdateTo(pendingSteps.end());
    }

    void UpdateTo(typename StepList::iterator newInvalidStateBegin)
    {
        if(invalidStateBegin == pendingSteps.end())
            return;

        if (newInvalidStateBegin != pendingSteps.end() &&
            (*newInvalidStateBegin)->GetTime() <= (*invalidStateBegin)->GetTime())
            return;

        auto state = std::ref((*std::prev(invalidStateBegin))->GetFinishedState());
        for (; invalidStateBegin != newInvalidStateBegin; invalidStateBegin++)
        {
            state = std::ref((*invalidStateBegin)->Run(state));
            assert(!state.get().State.array().isNaN().any());
            assert(!state.get().Covariance.array().isNaN().any());
        }
    }

  private:
    StepList pendingSteps;
    TimePointType predictEndTime;
    typename StepList::iterator invalidStateBegin;

    std::array<typename StepList::iterator, ParallelUpdateCount + 1> lastUpdatePos; // first one for predict.

    typename StepList::iterator findBeforePos(TimePointType time)
    {
        auto insertPos = prev(pendingSteps.end());
        while ((*insertPos)->GetTime() > time)
            insertPos--;

        return insertPos;
    }

    void dropHistory()
    {
        auto earestUpdatedPos = *std::min_element(lastUpdatePos.begin(), lastUpdatePos.end(),
                                                  [](auto &a, auto &b) { return (*a)->GetTime() < (*b)->GetTime(); });
        UpdateTo(next(earestUpdatedPos));
        pendingSteps.erase(pendingSteps.begin(), earestUpdatedPos);
    }

    template <int UpdateIndex>
    void doInsert(typename StepList::iterator beforePos, StepPtr &&newStep)
    {
        auto &beforeStep = *beforePos;
        auto afterPos = std::next(beforePos);
        auto &afterStep = *afterPos;
        auto insertedPos = this->pendingSteps.insert(afterPos, std::move(newStep));
        auto &insertedStep = *insertedPos;

        beforeStep->SetDuration(insertedStep->GetTime() - beforeStep->GetTime() );
        TimePointType endTime;
        if (afterPos == pendingSteps.end())
        {
            predictEndTime = max(predictEndTime, insertedStep->GetTime());
            endTime = predictEndTime;
        }
        else
        {
            predictEndTime = afterStep->GetTime();
        }
        insertedStep->SetDuration(endTime - insertedStep->GetTime());

        if (invalidStateBegin == pendingSteps.end() || (*invalidStateBegin)->GetTime() > insertedStep->GetTime())
            invalidStateBegin = insertedPos;

        lastUpdatePos[UpdateIndex] = insertedPos;
        dropHistory();
    }
};
}

#endif
