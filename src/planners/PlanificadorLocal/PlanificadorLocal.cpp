#include "planners/PlanificadorLocal.h"
#include "ompl/control/SpaceInformation.h"

ompl::guillermo::PlanificadorLocal::PlanificadorLocal(const SpaceInformation *si, unsigned int k) :
    DirectedControlSampler(si), cs_(si->allocControlSampler()), numControlSamples_(k)
{
}

ompl::guillermo::PlanificadorLocal::~PlanificadorLocal()
{
}

unsigned int ompl::guillermo::PlanificadorLocal::sampleTo(Control *control, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest, NULL);
}

unsigned int ompl::guillermo::PlanificadorLocal::sampleTo(Control *control, const Control *previous, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

unsigned int ompl::guillermo::PlanificadorLocal::getBestControl (Control *control, const base::State *source, base::State *dest)
{
    // Sample the first control
    cs_->sample(control, source);

    const unsigned int minDuration = si_->getMinControlDuration();
    const unsigned int maxDuration = si_->getMaxControlDuration();

    unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
    // Propagate the first control, and find how far it is from the target state
    base::State *bestState   = si_->allocState();
    steps = si_->propagateWhileValid(source, control, steps, bestState);

    if (numControlSamples_ > 1)
    {
        Control     *tempControl = si_->allocControl();
        base::State *tempState   = si_->allocState();
        double bestDistance      = si_->distance(bestState, dest);

        // Sample k-1 more controls, and save the control that gets closest to target
        for (unsigned int i = 1; i < numControlSamples_; ++i)
        {
            unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
            cs_->sample(tempControl, source);

            sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
            double tempDistance = si_->distance(tempState, dest);
            if (tempDistance < bestDistance)
            {
                si_->copyState(bestState, tempState);
                si_->copyControl(control, tempControl);
                bestDistance = tempDistance;
                steps = sampleSteps;
            }
        }

        si_->freeState(tempState);
        si_->freeControl(tempControl);
    }

    si_->copyState(dest, bestState);
    si_->freeState(bestState);

    return steps;
}
