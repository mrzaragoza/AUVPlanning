#ifndef AUV_GOAL_STATE_
#define AUV_GOAL_STATE_

#include <math.h> 
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
  	namespace auvplanning
  	{
		class AUVGoalState : public base::GoalState
		{
		public:
	    	AUVGoalState(const base::SpaceInformationPtr &si) : base::GoalState(si)
	    	{
	    	}

			virtual double 	distanceGoal(const base::State *st) const;
		};
	}
}

#endif /* AUV_GOAL_STATE_ */