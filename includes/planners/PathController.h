#ifndef OMPL_CONTROL_PATH_CONTROLLER_
#define OMPL_CONTROL_PATH_CONTROLLER_

#include "ompl/control/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "ompl/geometric/PathGeometric.h"
#include "robots/control/Controller.h"
#include "robots/control/AUV2StepPID.h"
#include <vector>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond
    }

    namespace auvplanning
    {
        class PathController : public base::Path
        {
        public:

            PathController(const base::SpaceInformationPtr &si);

            PathController(const PathController &path);

            virtual ~PathController()
            {
                freeMemory();
            }

            PathController& operator=(const PathController &other);

            virtual base::Cost cost(const base::OptimizationObjectivePtr& obj) const;

            virtual double length() const;

            virtual bool check() const;

            virtual void print(std::ostream &out) const;

            virtual void printAsMatrix(std::ostream &out) const;

            geometric::PathGeometric asGeometric() const;

            void append(const base::State *state);

            void append(const base::State *state, const base::State *reference, double duration);

            void interpolate();

            std::vector<base::State*>& getCurrentStates()
            {
                return currentStates_;
            }

            std::vector<base::State*>& getReferenceStates()
            {
                return referenceStates_;
            }

            std::vector<double>& getControlDurations()
            {
                return controlDurations_;
            }

            base::State* getCurrentState(unsigned int index)
            {
                return currentStates_[index];
            }

            const base::State* getCurrentState(unsigned int index) const
            {
                return currentStates_[index];
            }

            base::State* getReferenceState(unsigned int index)
            {
                return referenceStates_[index];
            }

            const base::State* getReferenceState(unsigned int index) const
            {
                return referenceStates_[index];
            }

            double getControlDuration(unsigned int index) const
            {
                return controlDurations_[index];
            }

            std::size_t getStateCount() const
            {
                return currentStates_.size();
            }

            std::size_t getReferenceStateCount() const
            {
                return referenceStates_.size();
            }

        protected:

            std::vector<base::State*>   currentStates_;

            std::vector<base::State*>   referenceStates_;

            std::vector<double>         controlDurations_;

            void freeMemory();

            void copyFrom(const PathController &other);

        };

    }
}

#endif
