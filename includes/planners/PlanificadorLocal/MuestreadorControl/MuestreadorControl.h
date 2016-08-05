#ifndef OMPL_MUESTREADOR_CONTROL_
#define OMPL_MUESTREADOR_CONTROL_

#include "ompl/base/State.h"
#include "ompl/base/spaces/RealVectorSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/util/RandomNumbers.h"
#include <vector>

namespace ompl
{
    namespace guillermo
    {

        class MuestreadorControl : control::ControlSampler
        {
        public:

            MuestreadorControl(const ControlSpace *space) : space_(space), maxDiferenciaControl(0.5),
                tiempoBase(50), controlZEstable(0.07)/*, goal_(new base::State()*/)
            {
                goal_->as<base::RealVectorStateSpace::StateType>()->values[0] = 0.0;
                goal_->as<base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
                goal_->as<base::RealVectorStateSpace::StateType>()->values[2] = 0.0;
                goal_->as<base::RealVectorStateSpace::StateType>()->values[3] = 0.0;
                goal_->as<base::RealVectorStateSpace::StateType>()->values[4] = 0.0;
                goal_->as<base::RealVectorStateSpace::StateType>()->values[5] = 0.0;
                goal_->as<base::RealVectorStateSpace::StateType>()->values[6] = 0.0;
                goal_->as<base::RealVectorStateSpace::StateType>()->values[7] = 0.0;
            }

            virtual ~MuestreadorControl()
            {
                delete(goal_);
            }

            virtual void sample(Control *control);

            virtual void sample(Control *control, const base::State *state);
            
            virtual void sampleNext(Control *control, const Control *previous);

            virtual void sampleNext(Control *control, const Control *previous, const base::State *state);

            virtual unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps);

            virtual void setGoal(const base::State *state);

            virtual base::State* getGoal();

        protected:

            /** \brief The control space this sampler operates on */
            const ControlSpace    *space_;

            /** \brief Instance of random number generator */
            RNG                    rng_;

            base::State            goal_;

            unsigned double        maxDiferenciaControl;
            unsigned int           tiempoBase;
            float                  controlZEstable;
        };

    }
}


#endif
