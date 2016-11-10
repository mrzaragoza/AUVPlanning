#ifndef OMPL_AUV_SEMIRANDOM_DIRECTED_CONTROL_SAMPLER_
#define OMPL_AUV_SEMIRANDOM_DIRECTED_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/ControlSpace.h"
#include "planners/PlanificadorLocal/MuestreadorControl/MuestreadorControl.h"
#include "yaml-cpp/yaml.h"

namespace ompl
{
    namespace auvplanning
    {

        class AUVSemiRandomDirectedControlSampler : public control::DirectedControlSampler
        {
        public:
            AUVSemiRandomDirectedControlSampler(const control::SpaceInformation *si, unsigned int k = 1, YAML::Node config = YAML::LoadFile("test.yaml"));

            ~AUVSemiRandomDirectedControlSampler();

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
