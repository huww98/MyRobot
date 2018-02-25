#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <memory>
#include "step.h"

namespace kf
{
template <int stateCount>
class KalmanFilter
{
  public:
    using StepType = Step<stateCount>;
    using StepPtr = std::unique_ptr<StepType>;
    using PredictStepType = PredictStepBase<stateCount>;
    using UpdateStepType = UpdateStepBase<stateCount>;

    void Predict(std::unique_ptr<PredictStepType> &&newStep)
    {
        auto pos = insertStep(std::move(newStep));
        UpdateLatestState(pos, false);
    }

    void Update(std::unique_ptr<UpdateStepType> &&newStep, bool dropHistory)
    {
        auto pos = insertStep(std::move(newStep));
        auto beforePos = pos;
        beforePos--;
        if (auto pStep = dynamic_cast<PredictStepType *>(beforePos.get()))
        {
            pos->SetPredictStep(pStep->Clone());
        }
        else if (auto uStep = dynamic_cast<UpdateStepType *>(beforePos.get()))
        {
            pos->SetPredictStep(uStep->GetPredictStep()->Clone());
        }
        UpdateLatestState(pos, dropHistory);
    }

    const State<stateCount> &GetLatestState() const
    {
        auto &lastestStep = *pendingSteps.rbegin();
        return lastestStep->GetFinishedState();
    }

  private:
    using StepList = std::list<StepPtr>;
    StepList pendingSteps;

    typename StepList::iterator insertStep(StepPtr &&newStep, bool dropHistory = false)
    {
        auto insertPos = pendingSteps.end();
        while (insertPos->GetTime() > newStep->GetTime())
            insertPos--;
        return pendingSteps.insert(insertPos, newStep);
    }

    void UpdateLatestState(typename StepList::iterator startPos, bool dropHistory)
    {
        auto beforeStartPos = startPos;
        beforeStartPos--;
        auto afterStartPos = startPos;
        afterStartPos++;

        startPos->UpdateParameters(startPos->GetTime() - beforeStartPos->GetTime());
        afterStartPos->UpdateParameters(afterStartPos->GetTime() - startPos->GetTime());

        auto state = beforeStartPos->GetFinishedState();
        for (auto runningPos = startPos; runningPos != pendingSteps.end(); runningPos++)
            state = runningPos->Run(state);

        if (dropHistory)
        {
            pendingSteps.erase(pendingSteps.begin(), startPos);
        }
    }
};
}

#endif
