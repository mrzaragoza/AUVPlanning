#ifndef OMPL_CONTROL_SIMPLE_DIRECTED_CONTROL_SAMPLER_
#define OMPL_CONTROL_SIMPLE_DIRECTED_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/ControlSpace.h"
#include "planners/PlanificadorLocal/MuestreadorControl/MuestreadorControl.h"

namespace ompl
{
    namespace guillermo
    {

        class PlanificadorLocal : public control::DirectedControlSampler
        {
        public:
            PlanificadorLocal(const control::SpaceInformation *si, unsigned int k = 1, base::State *start=NULL, base::State *goal=NULL);

            ~PlanificadorLocal();

            static ompl::control::DirectedControlSamplerPtr PlanificadorLocalAllocator(const ompl::control::SpaceInformation *si, unsigned int k, base::State *start, base::State *goal);

            unsigned int getNumControlSamples () const
            {
                return numControlSamples_;
            }

            void setNumControlSamples (unsigned int numSamples)
            {
                numControlSamples_ = numSamples;
            }

            unsigned int sampleTo(control::Control *control, const base::State *source, base::State *dest);

            unsigned int sampleTo(control::Control *control, const control::Control *previous, const base::State *source, base::State *dest);

            inline void setGoal(base::State *state){
                //cs_->setGoal(state);
                mc_->setGoal(state);
            }

            inline ompl::base::State* getGoal(){
                //return cs_->getGoal();
                return mc_->getGoal();
            }

            inline void setStart(base::State *state){
                //cs_->setStart(state);
                mc_->setStart(state);
            }

            inline ompl::base::State* getStart(){
                //return cs_->getStart();
                return mc_->getStart();
            }

            static ompl::control::ControlSamplerPtr MuestreadorControlAllocator(const ompl::control::ControlSpace *sp);

        protected:

            unsigned int getBestControl (control::Control *control, const base::State *source, base::State *dest);

            control::ControlSpacePtr         csp_;
            control::ControlSamplerPtr        cs_;

            static MuestreadorControl                *mc_;
            ompl::control::ControlSamplerAllocator        mca;

            /** \brief The number of controls to sample when finding the best control*/
            unsigned int            numControlSamples_;

        };

    }
}


#endif
