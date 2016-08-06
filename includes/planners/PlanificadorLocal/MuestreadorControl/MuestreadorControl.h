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
    namespace guillermo
    {

        class MuestreadorControl : control::ControlSampler
        {
        public:

            MuestreadorControl(const ompl::control::ControlSpace *space);

            virtual ~MuestreadorControl(){
            }

            virtual void sample(control::Control *control);

            virtual void sample(control::Control *control, const base::State *state);
            
            virtual void sampleNext(control::Control *control, const control::Control *previous);

            virtual void sampleNext(control::Control *control, const control::Control *previous, const base::State *state);

            virtual unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps);

            virtual void setGoal(base::State *state);

            virtual ompl::base::State* getGoal();

        private:

            /** \brief Instance of random number generator */
            RNG                             rng_;

            base::State                     *goal_;

            double                          maxDiferenciaControl;
            double                          margenCercania;
            unsigned int                    tiempoBase;
            float                           controlZEstable;
        };

    }
}


#endif
