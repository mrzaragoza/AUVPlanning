#include <math.h> 
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


class AUVGoal : public ompl::base::GoalState
{
public:
    AUVGoal(const ompl::base::SpaceInformationPtr &si) : ompl::base::GoalState(si)
    {
    }
    virtual bool isSatisfied(const ompl::base::State *st) const;
    virtual bool isSatisfied(const ompl::base::State *st, double *distance) const;
    virtual void setMaxDistance(double distance);

private:

    double      maxDistance_;
};