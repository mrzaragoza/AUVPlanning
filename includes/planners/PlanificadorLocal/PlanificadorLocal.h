#ifndef OMPL_CONTROL_SIMPLE_DIRECTED_CONTROL_SAMPLER_
#define OMPL_CONTROL_SIMPLE_DIRECTED_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/ControlSampler.h"

namespace ompl
{
    namespace guillermo
    {

        class PlanificadorLocal : public DirectedControlSampler
        {
        public:
            PlanificadorLocal(const SpaceInformation *si, unsigned int k = 1);

            virtual ~PlanificadorLocal();

            unsigned int getNumControlSamples () const
            {
                return numControlSamples_;
            }

            void setNumControlSamples (unsigned int numSamples)
            {
                numControlSamples_ = numSamples;
            }

            virtual unsigned int sampleTo(Control *control, const base::State *source, base::State *dest);

            virtual unsigned int sampleTo(Control *control, const Control *previous, const base::State *source, base::State *dest);

        protected:

            virtual unsigned int getBestControl (Control *control, const base::State *source, base::State *dest);

            ControlSamplerPtr       cs_;

            /** \brief The number of controls to sample when finding the best control*/
            unsigned int            numControlSamples_;

        };

    }
}


#endif
