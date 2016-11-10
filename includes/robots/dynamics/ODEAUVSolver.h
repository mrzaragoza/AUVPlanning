#ifndef OMPL_AUVPLANNING_ODEAUVSOLVER_
#define OMPL_AUVPLANNING_ODEAUVSOLVER_

#include <ompl/control/ODESolver.h>
#include "ompl/control/Control.h"
#include "ompl/control/SpaceInformation.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <boost/version.hpp>
#if BOOST_VERSION >= 105300
#include <boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::odeint;
#else
#include <omplext_odeint/boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::omplext_odeint;
#endif

namespace ompl
{
    namespace auvplanning
    {

        template <class Solver = odeint::runge_kutta4<control::ODESolver::StateType> >
        class ODEAUVSolver : public control::ODESolver
        {
        public:

            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and an optional integration step size - default is 0.01
            ODEAUVSolver (const control::SpaceInformationPtr &si, const control::ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep)
            {
            }

        protected:

            /// \brief Solve the ODE using boost::numeric::odeint.
            virtual void solve (control::ODESolver::StateType &state, const control::Control *control, const double duration) const{
                Solver solver;

                if(si_->getControlSpace()->isCompound()){

                    const ompl::control::CompoundControlSpace::ControlType *newControl = static_cast<const ompl::control::CompoundControlSpace::ControlType*>(control);

                    control::Control *control1 = si_->allocControl();
                    control1 = newControl->components[0];

                    control::Control *control2 = si_->allocControl();
                    control2 = newControl->components[1];

                    control::ODESolver::ODEFunctor odefunc1 (ode_, control1);

                    odeint::integrate_const (solver, odefunc1, state, 0.0, duration, intStep_);

                    control::ODESolver::ODEFunctor odefunc2 (ode_, control2);

                    odeint::integrate_const (solver, odefunc2, state, 0.0, duration, intStep_);
                }else{

                    ODESolver::ODEFunctor odefunc (ode_, control);
                    odeint::integrate_const (solver, odefunc, state, 0.0, duration, intStep_);
                }

            }
        };
    }
}


#endif /* OMPL_AUVPLANNING_ODEAUVSOLVER_ */