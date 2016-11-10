#ifndef OMPL_CONTROLLER_
#define OMPL_CONTROLLER_

#include "ompl/control/ControlSampler.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "robots/dynamics/AUVdynamics.h"
#include <ompl/control/ODESolver.h>
#include "yaml-cpp/yaml.h"
#include <math.h> 

using namespace std;
using namespace ompl;

namespace ompl
{
    namespace controller
    {

        class Controller
        {
        public:
            Controller(const control::SpaceInformation *si);

            ~Controller(){}           

            unsigned int propagateController(const base::State *source, base::State *dest, double steps);
            unsigned int propagateWhileValidController(const base::State *source, base::State *dest, double steps);

        protected:

            unsigned int propagation(const base::State *source, const base::State *dest, double steps, bool checkValidity) = 0;
            double pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw) = 0;

            control::SpaceInformationPtr    sinf;
            auvplanning::AUVDynamicsPtr     dynamics_;
            control::ODESolverPtr           ode_;
            double                          stepSize;
            control::StatePropagatorPtr     stPropagator;

        };

    }
}


#endif
