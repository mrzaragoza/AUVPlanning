#include "planners/PlanificadorLocal/AUVSemiRandomDirectedControlSampler.h"

ompl::auvplanning::AUVSemiRandomDirectedControlSampler::AUVSemiRandomDirectedControlSampler(const control::SpaceInformation *si, unsigned int k) :
control::DirectedControlSampler(si), numControlSamples_(k), csp_(si->getControlSpace())
{        
    YAML::Node robot_config = YAML::LoadFile("../includes/robots/torpedo.yaml");
   
    controlZEstable = robot_config["torpedo/controlZEstable"].as<double>();
}

ompl::auvplanning::AUVSemiRandomDirectedControlSampler::~AUVSemiRandomDirectedControlSampler()
{
}


unsigned int ompl::auvplanning::AUVSemiRandomDirectedControlSampler::sampleTo(control::Control *control, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

unsigned int ompl::auvplanning::AUVSemiRandomDirectedControlSampler::sampleTo(control::Control *control, const control::Control *previous, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

void ompl::auvplanning::AUVSemiRandomDirectedControlSampler::sampleControl(control::Control *control, const base::State * state, const base::State *dest)
{

    const unsigned int dim =3;

    const base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(csp_.get())->getBounds();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::RealVectorControlSpace::ControlType*>(control);

    //Distancias con estado actual
    double diff_x               = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - 
                                        state->as<base::RealVectorStateSpace::StateType>()->values[0];
    double diff_y               = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - 
                                        state->as<base::RealVectorStateSpace::StateType>()->values[1];
    double diff_z               = dest->as<base::RealVectorStateSpace::StateType>()->values[2] - 
                                        state->as<base::RealVectorStateSpace::StateType>()->values[2];

    double maxDiferenciaControl = 0.5;
    double margenCercania = 0.2;

    //Calculo del ángulo que hay entre el estado actual y el final y la diferencia entre éste y el yaw del vehículo
    double heading = atan2(diff_y,diff_x); //radianes
    double diff_heading = heading - state->as<base::RealVectorStateSpace::StateType>()->values[3];


    double num_aleatorio = (double) std::rand() / RAND_MAX;

    if(diff_heading > margenCercania){
        double aw= num_aleatorio + ((double) std::rand() / RAND_MAX)*maxDiferenciaControl/2;
        newControl->values[1] = aw;
        double ay= num_aleatorio - ((double) std::rand() / RAND_MAX)*maxDiferenciaControl/2;
        newControl->values[2] = ay;

    }else if(diff_heading < -margenCercania){   
        double aq= num_aleatorio - ((double) std::rand() / RAND_MAX)*maxDiferenciaControl/2;
        newControl->values[1] = aq;
        double as= num_aleatorio + ((double) std::rand() / RAND_MAX)*maxDiferenciaControl/2;
        newControl->values[2] = as;

    }else{ 
        double ad= num_aleatorio + (((double) std::rand() / RAND_MAX)*2-1)*maxDiferenciaControl/8;
        newControl->values[1] = ad;
        double af= num_aleatorio + (((double) std::rand() / RAND_MAX)*2-1)*maxDiferenciaControl/8;
        newControl->values[2] = af;
    }

    if(diff_z > margenCercania){
        double ax = ((double) std::rand() / RAND_MAX);
        newControl->values[0] = ax;

    }else if(diff_z < -margenCercania){
        double ax = ((double) std::rand() / RAND_MAX)-1;
        newControl->values[0] = ax;

    }else{ 
        double ax = (((double) std::rand() / RAND_MAX)*2-1) * 0.1 + controlZEstable;
        newControl->values[0] = ax;
    }

    //Se multiplica por la normalización de la distancia, haciendo que cuanto
    //más cerca se esté del objetivo, los controles sean más pequeños.

    if(newControl->values[0] > 1) newControl->values[0] = 1;
    else if(newControl->values[0] < -1) newControl->values[0] = -1;
    
    if(newControl->values[1] > 1) newControl->values[1] = 1;
    else if(newControl->values[1] < -1) newControl->values[1] = -1;
    
    if(newControl->values[2] > 1) newControl->values[2] = 1;
    else if(newControl->values[2] < -1) newControl->values[2] = -1;
  
}

unsigned int ompl::auvplanning::AUVSemiRandomDirectedControlSampler::getBestControl(control::Control *control, const base::State *source, base::State *dest)
{

    // Sample the first control
    sampleControl(control, source, dest);

    const unsigned int minDuration = si_->getMinControlDuration();
    const unsigned int maxDuration = si_->getMaxControlDuration();

    unsigned int steps = rng_.uniformInt(minDuration, maxDuration);
    // Propagate the first control, and find how far it is from the target state
    base::State *bestState   = si_->allocState();
    steps = si_->propagateWhileValid(source, control, steps, bestState);

    if (numControlSamples_ > 1)
    {
        control::Control     *tempControl = si_->allocControl();
        base::State *tempState   = si_->allocState();
        double bestDistance      = si_->distance(bestState, dest);

        // Sample k-1 more controls, and save the control that gets closest to target
        for (unsigned int i = 1; i < numControlSamples_; ++i)
        {
            unsigned int sampleSteps = rng_.uniformInt(minDuration, maxDuration);
            sampleControl(tempControl, source, dest);

            sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
            double tempDistance = si_->distance(tempState, dest);
            if (tempDistance < bestDistance)
            {
                si_->copyState(bestState, tempState);
                si_->copyControl(control, tempControl);
                bestDistance = tempDistance;
                steps = sampleSteps;
            }
        }

        si_->freeState(tempState);
        si_->freeControl(tempControl);
    }

    si_->copyState(dest, bestState);
    si_->freeState(bestState);

    return steps;
}

