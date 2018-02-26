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
    using PredictStepPtr = std::unique_ptr<PredictStepType>;
    using UpdateStepType = UpdateStepBase<stateCount>;
    using UpdateStepPtr = std::unique_ptr<UpdateStepType>;

    KalmanFilter(StepPtr &&firstStep)
    {
        pendingSteps.push_back(std::move(firstStep));
    }

    void Predict(PredictStepPtr &&newStep)
    {
        auto pos = insertStep(std::move(newStep));
        UpdateLatestState(pos, false);
    }

    void Update(UpdateStepPtr &&newStep, bool dropHistory)
    {
        UpdateStepType *newStepPtr;
        auto pos = insertStep(std::move(newStep));
        auto beforePos = pos;
        beforePos--;
        if (auto pStep = dynamic_cast<PredictStepType *>((*beforePos).get()))
        {
            newStepPtr->SetPredictStep(PredictStepPtr(pStep->Clone()));
        }
        else if (auto uStep = dynamic_cast<UpdateStepType *>((*beforePos).get()))
        {
            newStepPtr->SetPredictStep(PredictStepPtr(uStep->GetPredictStep()->Clone()));
        }
        else
        {
            assert(false);
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
        while ((*insertPos)->GetTime() > newStep->GetTime())
            insertPos--;
        return pendingSteps.insert(insertPos, std::move(newStep));
    }

    void UpdateLatestState(typename StepList::iterator startPos, bool dropHistory)
    {
        auto beforeStartPos = startPos;
        beforeStartPos--;
        auto afterStartPos = startPos;
        afterStartPos++;

        auto &beforeStartPtr = *beforeStartPos;
        auto &startPtr = *startPos;
        auto &afterStartPtr = *afterStartPos;

        startPtr->GenerateParameters(startPtr->GetTime() - beforeStartPtr->GetTime());
        afterStartPtr->GenerateParameters(afterStartPtr->GetTime() - startPtr->GetTime());

        auto state = beforeStartPtr->GetFinishedState();
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
