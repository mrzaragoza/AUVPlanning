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

        protected:

            unsigned int propagation(control::Control *control, const base::State *source, base::State *dest);
            double pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw);

            void isPIDResetNeeded(const base::State *init, const base::State *dest);

            /** \brief The number of controls to sample when finding the best control*/
            control::SpaceInformationPtr    sinf;
            auvplanning::AUVDynamicsPtr     dynamics_;
            control::ODESolverPtr           ode_;
            double                          stepSize;
            control::StatePropagatorPtr     stPropagator;


            double                          Kpz, Kdz, Kiz;
            double                          Kpsurge, Kdsurge, Kisurge;
            double                          Kpyaw, Kdyaw, Kiyaw;

            double                          mass;
            double                          c_rbm1, c_rbm3, c_rbm4;
            double                          c_am1, c_am3, c_am4;
            double                          c_ld1, c_ld3, c_ld4;
            double                          c_qd1, c_qd3, c_qd4;
            double                          controlZEstable;
            double                          l_motores;
            double                          max_fuerza_motores;
            double                          dist_z_inicial;

            double                          porcentaje_dist_rango_objetivo;
            double                          rango_min_objetivo;
            double                          rango_max_objetivo;
            double                          porcentaje_dist_profundidad_rango_objetivo;
            double                          rango_profundidad_min_objetivo;
            double                          rango_profundidad_max_objetivo;

            const double                    rango_dist_objetivo = 0.5;
            const double                    rango_profundidad_objetivo = 0.5;
            const double                    rango_yaw_objetivo = 0.005;

            double                          dist_inicial = 0;
            //double                          heading = 0;

            double                          pre_errorz = 0;
            double                          pre_errorsurge = 0;
            double                          pre_erroryaw = 0;
            double                          integralz = 0;
            double                          integralsurge = 0;
            double                          integralyaw = 0;

            //base::State*                    inicial;
            base::State*                    reference;

        };

    }
}


#endif
