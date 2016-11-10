#ifndef OMPL_MUESTREADOR_CONTROLPID_
#define OMPL_MUESTREADOR_CONTROLPID_

#include "ompl/base/State.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSpace.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl/util/RandomNumbers.h"
#include <math.h>
#include <vector>
#include "yaml-cpp/yaml.h"

namespace ompl
{
    namespace auvplanning
    {

        class MuestreadorControlPID : public control::ControlSampler
        {
        public:

            MuestreadorControlPID(const ompl::control::ControlSpace *space, YAML::Node config = YAML::LoadFile("test.yaml"));

            ~MuestreadorControlPID(){
            }

            void sample(control::Control *control);

            void sample(control::Control *control, const base::State *state);
            
            void sampleNext(control::Control *control, const control::Control *previous);

            void sampleNext(control::Control *control, const control::Control *previous, const base::State *state);

            unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps);

            void setGoal(base::State *state);

            ompl::base::State* getGoal();

        private:

            RNG                             rng_;
            static base::State              *goal_;
            const control::ControlSpace     *space_mc;

            double                          Kpz, Kdz, Kiz;
            double                          Kpsurge, Kdsurge, Kisurge;
            double                          Kpyaw, Kdyaw, Kiyaw;
            double                          pre_errorz, pre_errorsurge, pre_erroryaw;
            double                          integralz, integralsurge, integralyaw;

            double                          mass;
            double                          c_rbm1, c_rbm3, c_rbm4;
            double                          c_am1, c_am3, c_am4;
            double                          c_ld1, c_ld3, c_ld4;
            double                          c_qd1, c_qd3, c_qd4;
            double                          controlZEstable;
            double                          l_motores;
            double                          max_fuerza_motores;
            double                          dist_xy_inicial;
            double                          dist_z_inicial;
            double                          rango_dist_objetivo;
            control::Control                controlTemporal;

            double                          porcentaje_dist_rango_objetivo;
            double                          rango_min_objetivo;
            double                          rango_max_objetivo;
            double                          porcentaje_dist_profundidad_rango_objetivo;
            double                          rango_profundidad_min_objetivo;
            double                          rango_profundidad_max_objetivo;

            void twoStepsPID(control::Control *control, const base::State *state);

        };

    }
}


#endif
