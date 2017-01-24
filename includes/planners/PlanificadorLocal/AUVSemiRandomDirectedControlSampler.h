#ifndef OMPL_AUV_SEMIRANDOM_DIRECTED_CONTROL_SAMPLER_
#define OMPL_AUV_SEMIRANDOM_DIRECTED_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/ControlSpace.h"
#include "yaml-cpp/yaml.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{
    namespace auvplanning
    {

        class AUVSemiRandomDirectedControlSampler : public control::DirectedControlSampler
        {
        public:
            AUVSemiRandomDirectedControlSampler(const control::SpaceInformation *si, unsigned int k = 1);

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

        protected:

            void sampleControl(control::Control *control, const base::State * state, const base::State *dest);
            unsigned int getBestControl (control::Control *control, const base::State *source, base::State *dest);

            control::ControlSpacePtr         csp_;

            unsigned int                     numControlSamples_;
            ompl::RNG                        rng_;
            double                           controlZEstable;

        };

    }
}


#endif

