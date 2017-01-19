#include "robots/control/Controller.h"
#include <ompl/control/ODESolver.h>
#include "robots/dynamics/AUVdynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace oauv = ompl::auvplanning;

ompl::controller::Controller::Controller(const control::SpaceInformation *si)
{
    si_ = si;
    stepSize = si->getPropagationStepSize();
    stPropagator = si->getStatePropagator();
}

ompl::controller::Controller::~Controller(){
}

unsigned int ompl::controller::Controller::propagateController(const base::State *source, base::State *dest, unsigned int steps)
{
    return propagation(source, dest, steps, false);
}

unsigned int ompl::controller::Controller::propagateWhileValidController(const base::State *source, base::State *dest, unsigned int steps)
{
    return propagation(source, dest, steps, true);
}

void ompl::controller::Controller::propagateController(const base::State *source, const base::State *dest, std::vector<base::State*> &result, unsigned int steps, bool alloc)
{
    //printf("\tController numero de steps de entrada: %d\n", steps);
    if (alloc)
    {
        result.resize(steps);
        for (unsigned int i = 0 ; i < result.size() ; ++i){
            result[i] = si_->allocState();
            si_->copyState(result[i], dest);
        }
    }
    else
    {
        if (result.empty())
            return;
        steps = std::min((int)steps, (int)result.size());
    }

    int st = 0;
    int step = 1;

    if (st < steps)
    {
        //propagation(source, result[st], stepSize, false);
        propagation(source, result[st], step, false);
        ++st;

        while (st < steps)
        {
            //propagation(result[st-1], result[st], stepSize, false);
            propagation(result[st-1], result[st], step, false);
            ++st;
        }
    }
}