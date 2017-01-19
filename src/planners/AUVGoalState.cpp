#include <planners/AUVGoalState.h>


double ompl::auvplanning::AUVGoalState::distanceGoal(const base::State *st) const{ 

    //printf("LLamada a AUVGoalState::distanceGoal\n");
    double x   = state_->as<base::RealVectorStateSpace::StateType>()->values[0] - st->as<base::RealVectorStateSpace::StateType>()->values[0];
    double y   = state_->as<base::RealVectorStateSpace::StateType>()->values[1] - st->as<base::RealVectorStateSpace::StateType>()->values[1];
    double z   = state_->as<base::RealVectorStateSpace::StateType>()->values[2] - st->as<base::RealVectorStateSpace::StateType>()->values[2];
    double yaw = state_->as<base::RealVectorStateSpace::StateType>()->values[3] - st->as<base::RealVectorStateSpace::StateType>()->values[3];

    double dist = sqrt ( pow (x,2) + pow (y,2) + pow (z,2) + pow (yaw,2) );
    return dist;
}