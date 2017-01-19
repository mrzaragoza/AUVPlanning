#ifndef OMPL_AUV_TWO_STEP_PID_CONTROL_SAMPLER_
#define OMPL_AUV_TWO_STEP_PID_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"
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
    namespace auvplanning
    {

        class AUV2StepPIDControlSampler : public control::DirectedControlSampler
        {
        public:
            AUV2StepPIDControlSampler(const control::SpaceInformation *si, YAML::Node config = YAML::LoadFile("test.yaml"));

            ~AUV2StepPIDControlSampler();

            unsigned int sampleTo(control::Control *control, const base::State *source, base::State *dest);
            unsigned int sampleTo(control::Control *control, const control::Control *previous, const base::State *source, base::State *dest);

            //unsigned int sampleToStates(control::Control *control, const base::State *source, base::State *dest, std::vector<base::State*> istates);
        protected:

            unsigned int propagation(control::Control *control, const base::State *source, base::State *dest/*, std::vector<base::State*> istates = NULL*/);
            double pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw);

            void isPIDResetNeeded(const base::State *init, const base::State *dest);

            /** \brief The number of controls to sample when finding the best control*/
            double                          stepSize;
            control::StatePropagatorPtr     stPropagator;


            double                          Kpz, Kdz, Kiz;
            double                          Kpsurge, Kdsurge, Kisurge;
            double                          Kpyaw, Kdyaw, Kiyaw;

            double                          controlZEstable;
            double                          l_motores;
            double                          max_fuerza_motores;
            double                          dist_z_inicial;

            const double                    rango_dist_objetivo = 0.5;
            const double                    rango_profundidad_objetivo = 0.5;
            const double                    rango_yaw_objetivo = 0.005;

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
