#ifndef OMPL_MUESTREADOR_CONTROL_
#define OMPL_MUESTREADOR_CONTROL_

#include "ompl/base/State.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSpace.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl/util/RandomNumbers.h"
#include <math.h>
#include <vector>

namespace ompl
{
    namespace auvplanning
    {

        class MuestreadorControl : public control::ControlSampler
        {
        public:

            MuestreadorControl(const ompl::control::ControlSpace *space);

            ~MuestreadorControl(){
            }

            void sample(control::Control *control);

            void sample(control::Control *control, const base::State *state);
            
            void sampleNext(control::Control *control, const control::Control *previous);

            void sampleNext(control::Control *control, const control::Control *previous, const base::State *state);

            unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps);

            void setGoal(base::State *state);

            ompl::base::State* getGoal();

            void setStart(base::State *state);

            ompl::base::State* getStart();

        private:

            RNG                             rng_;

            static base::State              *goal_;
            static base::State              *start_;

            double                          diff_x_inicial;
            double                          diff_y_inicial;
            double                          diff_z_inicial;
            double                          dist_total;
            double                          dist_total_3D;

            double                          maxDiferenciaControl;
            double                          margenCercania;
            unsigned int                    tiempoBase;
            float                           controlZEstable;

            const control::ControlSpace    *space_mc;
            int cont;
        };

    }
}


#endif
