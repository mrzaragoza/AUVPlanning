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
        OMPL_CLASS_FORWARD(Controller);

        class Controller
        {
        public:
            Controller(const control::SpaceInformation *si);

            ~Controller();          

            virtual unsigned int propagateController(const base::State *source, base::State *dest, unsigned int steps);
            virtual void propagateController(const base::State *source, const base::State *dest, std::vector<base::State*> &result, unsigned int steps, bool alloc);
            virtual unsigned int propagateWhileValidController(const base::State *source, base::State *dest, unsigned int steps);

        protected:

            virtual unsigned int propagation(const base::State *source, base::State *dest, unsigned int steps, bool checkValidity) = 0;
            virtual double pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw) = 0;

            //control::SpaceInformationPtr          sinf;
            const control::SpaceInformation         *si_;
            auvplanning::AUVDynamicsPtr             dynamics_;
            control::ODESolverPtr                   ode_;
            double                                  stepSize;
            control::StatePropagatorPtr             stPropagator;

        };

    }
}


#endif
