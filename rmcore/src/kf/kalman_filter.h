#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <memory>
#include "step.h"

namespace kf
{
template <int stateCount, typename StateT = State<stateCount>>
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

    static int constexpr StateCount = stateCount;

    KalmanFilter(StepPtr &&firstStep)
    {
        pendingSteps.push_back(std::move(firstStep));
    }

    void Predict(TimePointType time, PredictorPtr &&predictor)
    {
        auto beforePos = findBeforePos(time);
        doInsert(beforePos, std::make_unique<PredictStepType>(time, predictor), false);
    }

    void Update(TimePointType time, UpdaterPtr &&updater, bool dropHistory)
    {
        auto beforePos = findBeforePos(time);
        PredictorPtr predictor;

        if (auto pStep = dynamic_cast<PredictStepType *>((*beforePos).get()))
        {
            predictor = pStep->GetPredictor();
        }
        else if (auto uStep = dynamic_cast<UpdateStepType *>((*beforePos).get()))
        {
            predictor = uStep->GetPredictor();
        }
        else
        {
            assert(false);
        }

        doInsert(beforePos, std::make_unique<UpdateStepType>(time, predictor, std::move(updater)), false);
    }

    const StateType &GetLatestState() const
    {
        auto &lastestStep = *pendingSteps.rbegin();
        return lastestStep->GetFinishedState();
    }

  private:
    using StepList = std::list<StepPtr>;
    StepList pendingSteps;

    typename StepList::iterator findBeforePos(TimePointType time)
    {
        auto insertPos = pendingSteps.end();
        while ((*insertPos)->GetTime() > time)
            insertPos--;

        return insertPos;
    }

    void doInsert(typename StepList::iterator beforePos, StepPtr &&newStep, bool dropHistory)
    {
        auto beforeStartPos = beforePos;
        auto &beforeStartStep = *beforePos;
        auto afterStartPos = std::next(beforePos);
        auto &afterStartStep = *afterStartPos;
        auto startPos = this->pendingSteps.insert(afterStartPos, std::move(newStep));
        auto &startStep = *startPos;

        startStep->SetDuration(startStep->GetTime() - beforeStartStep->GetTime());
        if (afterStartPos != pendingSteps.end())
        {
            afterStartStep->SetDuration(afterStartStep->GetTime() - startStep->GetTime());
        }

        auto state = beforeStartStep->GetFinishedState();
        for (auto runningPos = startPos; runningPos != pendingSteps.end(); runningPos++)
            state = (*runningPos)->Run(state);

        if (dropHistory)
        {
            pendingSteps.erase(pendingSteps.begin(), startPos);
        }
    }
};
}

#endif
