#include <planners/AUVGoal.h>


bool AUVGoal::isSatisfied(const ompl::base::State *st) const{

    bool result = false;

    double x   = state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double y   = state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double z   = state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double yaw = state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];

    double dist = sqrt ( pow (x,2) + pow (y,2) + pow (z,2) + pow (yaw,2) );

    if (dist < maxDistance_) result = true;
    return result;
}

bool AUVGoal::isSatisfied(const ompl::base::State *st, double *distance) const{
    bool result = false;

    double x   = state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double y   = state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double z   = state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double yaw = state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] - st->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];

    double dist = sqrt ( pow (x,2) + pow (y,2) + pow (z,2) + pow (yaw,2) );

    if (dist < maxDistance_) result = true;

    if (distance != NULL) *distance = dist;
    return result;

}

void AUVGoal::setMaxDistance(double distance){
    maxDistance_ = distance;
}