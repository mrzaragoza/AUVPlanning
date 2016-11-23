#include "robots/control/Controller.h"
#include <ompl/control/ODESolver.h>
#include "robots/dynamics/AUVdynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace oauv = ompl::auvplanning;

ompl::controller::Controller::Controller(const control::SpaceInformation *si) :
sinf(si),
dynamics_(new oauv::AUVDynamics()),
ode_(new ompl::control::ODEBasicSolver<>(sinf, boost::bind(&oauv::AUVDynamics::ode, dynamics_, _1, _2, _3)))
{
    stepSize = si->getPropagationStepSize();
    stPropagator = oc::ODESolver::getStatePropagator(ode_, boost::bind(&oauv::AUVDynamics::postPropagate, dynamics_, _1, _2, _3, _4));

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
    if (alloc)
    {
        result.resize(steps);
        for (unsigned int i = 0 ; i < result.size() ; ++i){
            result[i] = sinf->allocState();
            sinf->copyState(result[i], dest);
        }
    }
    else
    {
        if (result.empty())
            return;
        steps = std::min((int)steps, (int)result.size());
    }

    int st = 0;

    if (st < steps)
    {
        propagation(source, result[st], stepSize, false);
        ++st;

        while (st < steps)
        {
            propagation(result[st-1], result[st], stepSize, false);
            ++st;
        }
    }
}