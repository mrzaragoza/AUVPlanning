#ifndef OMPL_AUV_TWO_STEP_PID_
#define OMPL_AUV_TWO_STEP_PID_

#include "robots/control/Controller.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "robots/dynamics/AUVdynamics.h"
#include <ompl/control/ODESolver.h>
#include "yaml-cpp/yaml.h"


using namespace std;
using namespace ompl;

namespace ompl
{
    namespace controller
    {

        class AUV2StepPID : public Controller
        {
        public:
            AUV2StepPID(const control::SpaceInformation *si);

            ~AUV2StepPID(){}            

        protected:

            unsigned int propagation(const base::State *source, base::State *dest, unsigned int steps, bool checkValidity);
            double pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw);
            void isPIDResetNeeded(const base::State *init, const base::State *dest);

            double                          Kpz, Kdz, Kiz;
            double                          Kpsurge, Kdsurge, Kisurge;
            double                          Kpyaw, Kdyaw, Kiyaw;

            double                          controlZEstable;
            double                          l_motores;
            double                          max_fuerza_motores;

            const double                    rango_dist_objetivo = 0.5;
            const double                    rango_profundidad_objetivo = 0.5;
            const double                    rango_yaw_objetivo = 0.001;

            double                          dist_inicial = 0;

            double                          pre_errorz = 0; 
            double                          pre_errorsurge = 0;
            double                          pre_erroryaw = 0;
            double                          integralz = 0;
            double                          integralsurge = 0;
            double                          integralyaw = 0;

            base::State*                    reference;

        };

    }
}


#endif
