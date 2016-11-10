#include "robots/control/Controller.h"
#include <ompl/control/ODESolver.h>
#include "robots/dynamics/AUVdynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace oauv = ompl::auvplanning;

ompl::controller::AUVPID::AUVPID(const control::SpaceInformation *si) :
sinf(si),
dynamics_(new oauv::AUVDynamics()),
ode_(new ompl::control::ODEBasicSolver<>(sinf, boost::bind(&oauv::AUVDynamics::ode, dynamics_, _1, _2, _3)))
{
    stepSize = si->getPropagationStepSize();
    stPropagator = oc::ODESolver::getStatePropagator(ode_, boost::bind(&oauv::AUVDynamics::postPropagate, dynamics_, _1, _2, _3, _4));

}

unsigned int ompl::controller::AUVPID::propagateController(const base::State *source, base::State *dest, double steps)
{
    return propagation(source, dest, steps, false);
}

unsigned int ompl::controller::AUVPID::propagateWhileValidController(const base::State *source, base::State *dest, double steps)
{
    return propagation(source, dest, steps, true);
}
