#ifndef OMPL_CONTROL_PLANNERS_DYNAMIC_RRT_
#define OMPL_CONTROL_PLANNERS_DYNAMIC_RRT_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "planners/PathController.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

namespace ompl
{

    namespace auvplanning
    {       

        /** \brief Rapidly-exploring Random Tree */
        class DynamicRRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            DynamicRRT(const control::SpaceInformationPtr &si);

            virtual ~DynamicRRT();

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            virtual void clear();

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
            {
                nn_.reset(new NN<Motion*>());
            }

            virtual void setup();

        protected:


            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion() : currentState(NULL), referenceState(NULL), steps(0), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const control::SpaceInformation *si) : currentState(si->allocState()), referenceState(si->allocState()), steps(0), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *currentState;

                /** \brief The final state used by the PID controller */
                base::State 	  *referenceState;

                /** \brief The number of steps the control is applied for */
                unsigned int       steps;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->currentState, b->currentState);
            }

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief Control sampler */
            control::DirectedControlSamplerPtr             controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const control::SpaceInformation                *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;
        };

    }
}

#endif
