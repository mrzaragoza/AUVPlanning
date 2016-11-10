#include "planners/PlanificadorLocal/AUVSemiRandomDirectedControlSampler.h"

ompl::auvplanning::MuestreadorControl * ompl::auvplanning::AUVSemiRandomDirectedControlSampler::AUVSemiRandomDirectedControlSampler::mc_ = NULL;

ompl::auvplanning::AUVSemiRandomDirectedControlSampler::AUVSemiRandomDirectedControlSampler(const control::SpaceInformation *si, unsigned int k, YAML::Node config) :
control::DirectedControlSampler(si), numControlSamples_(k), csp_(si->getControlSpace())
{
    mca = boost::bind(MuestreadorControlAllocator, csp_.get());    
    csp_->setControlSamplerAllocator(mca);
    if(mc_ == NULL)  mc_  = (MuestreadorControl * )si->allocControlSampler().get();

    base::State *stateGoal;
    base::State *stateStart;
    stateGoal = si->allocState();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[0] = config["general/goal"][0].as<double>();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[1] = config["general/goal"][1].as<double>();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[2] = config["general/goal"][2].as<double>();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[3] = config["general/goal"][3].as<double>();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[4] = config["general/goal"][4].as<double>();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[5] = config["general/goal"][5].as<double>();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[6] = config["general/goal"][6].as<double>();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[7] = config["general/goal"][7].as<double>();
    printf("%f %f %f %f %f %f %f %f \n",
        stateGoal->as<base::RealVectorStateSpace::StateType>()->values[0],
        stateGoal->as<base::RealVectorStateSpace::StateType>()->values[1],
        stateGoal->as<base::RealVectorStateSpace::StateType>()->values[2],
        stateGoal->as<base::RealVectorStateSpace::StateType>()->values[3],
        stateGoal->as<base::RealVectorStateSpace::StateType>()->values[4],
        stateGoal->as<base::RealVectorStateSpace::StateType>()->values[5],
        stateGoal->as<base::RealVectorStateSpace::StateType>()->values[6],
        stateGoal->as<base::RealVectorStateSpace::StateType>()->values[7]);
    setGoal(stateGoal);
    stateStart = si->allocState();
    stateStart->as<base::RealVectorStateSpace::StateType>()->values[0] = config["general/start"][0].as<double>();
    stateStart->as<base::RealVectorStateSpace::StateType>()->values[1] = config["general/start"][1].as<double>();
    stateStart->as<base::RealVectorStateSpace::StateType>()->values[2] = config["general/start"][2].as<double>();
    stateStart->as<base::RealVectorStateSpace::StateType>()->values[3] = config["general/start"][3].as<double>();
    stateStart->as<base::RealVectorStateSpace::StateType>()->values[4] = config["general/start"][4].as<double>();
    stateStart->as<base::RealVectorStateSpace::StateType>()->values[5] = config["general/start"][5].as<double>();
    stateStart->as<base::RealVectorStateSpace::StateType>()->values[6] = config["general/start"][6].as<double>();
    stateStart->as<base::RealVectorStateSpace::StateType>()->values[7] = config["general/start"][7].as<double>();
    printf("%f %f %f %f %f %f %f %f \n",
        stateStart->as<base::RealVectorStateSpace::StateType>()->values[0],
        stateStart->as<base::RealVectorStateSpace::StateType>()->values[1],
        stateStart->as<base::RealVectorStateSpace::StateType>()->values[2],
        stateStart->as<base::RealVectorStateSpace::StateType>()->values[3],
        stateStart->as<base::RealVectorStateSpace::StateType>()->values[4],
        stateStart->as<base::RealVectorStateSpace::StateType>()->values[5],
        stateStart->as<base::RealVectorStateSpace::StateType>()->values[6],
        stateStart->as<base::RealVectorStateSpace::StateType>()->values[7]);
    setStart(stateStart);
    }

ompl::auvplanning::AUVSemiRandomDirectedControlSampler::~AUVSemiRandomDirectedControlSampler()
{
}

ompl::control::ControlSamplerPtr ompl::auvplanning::AUVSemiRandomDirectedControlSampler::MuestreadorControlAllocator(const ompl::control::ControlSpace *sp)
{
    /*printf("AUVSemiRandomDirectedControlSampler::MuestreadorControlAllocator llamado\n");
    fflush(stdout);*/

    if(mc_ != NULL){
        /*printf("mc_ distinto de NULL\n");
        fflush(stdout);*/
        return ompl::control::ControlSamplerPtr(mc_);
    }
    else{
        /*printf("mc_ igual a NULL    \n");
        fflush(stdout);*/
        return ompl::control::ControlSamplerPtr(new MuestreadorControl(sp));
    }
}

unsigned int ompl::auvplanning::AUVSemiRandomDirectedControlSampler::sampleTo(control::Control *control, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

unsigned int ompl::auvplanning::AUVSemiRandomDirectedControlSampler::sampleTo(control::Control *control, const control::Control *previous, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

unsigned int ompl::auvplanning::AUVSemiRandomDirectedControlSampler::getBestControl (control::Control *control, const base::State *source, base::State *dest)
{

    /*printf("getBestControl Inicia!\n");
    fflush(stdout);

    std::vector<int> signature;
    csp_->computeSignature(signature);*/

    /*printf("ControlSpace signature: ");
    fflush(stdout);
    for(int i=0; i<signature.size();i++){
        printf("%d ",signature[i]);
        fflush(stdout);
    }
    printf("\n");
    fflush(stdout);*/

    // Sample the first control
    mc_->MuestreadorControl::sample(control, source);
            /*printf("control t1: %f\n", control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0]);
            printf("control t2: %f\n", control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1]);
            printf("control t3: %f\n", control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[2]);
            printf("source x: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[0]);
            printf("source y: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[1]);
            printf("source z: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[2]);
            printf("source yaw: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[3]);
            printf("source vx: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[4]);
            printf("source vy: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[5]);
            printf("source vz: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[6]);
            printf("source vyaw: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[7]);*/
    //cs_->sample(control, source);

    /*printf("getBestControl Ha hecho el sample\n");
    fflush(stdout);*/

    const unsigned int minDuration = si_->getMinControlDuration();
    const unsigned int maxDuration = si_->getMaxControlDuration();

    //unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
    unsigned int steps = mc_->MuestreadorControl::sampleStepCount(minDuration, maxDuration);
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
//            unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
            unsigned int sampleSteps = mc_->MuestreadorControl::sampleStepCount(minDuration, maxDuration);
//            cs_->sample(tempControl, source);
            /*printf("getBestControl haciendo otro sample\n");
            fflush(stdout);*/
            mc_->MuestreadorControl::sample(tempControl, source);
            /*printf("-tempControl t1: %f\n", tempControl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0]);
            printf("-tempControl t2: %f\n", tempControl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1]);
            printf("-tempControl t3: %f\n", tempControl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[2]);
            printf("-source x: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[0]);
            printf("-source y: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[1]);
            printf("-source z: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[2]);
            printf("-source yaw: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[3]);
            printf("-source vx: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[4]);
            printf("-source vy: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[5]);
            printf("-source vz: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[6]);
            printf("-source vyaw: %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[7]);
            printf("-tempState x: %f\n", tempState->as<base::RealVectorStateSpace::StateType>()->values[0]);
            printf("-tempState y: %f\n", tempState->as<base::RealVectorStateSpace::StateType>()->values[1]);
            printf("-tempState z: %f\n", tempState->as<base::RealVectorStateSpace::StateType>()->values[2]);
            printf("-tempState yaw: %f\n", tempState->as<base::RealVectorStateSpace::StateType>()->values[3]);
            printf("-tempState vx: %f\n", tempState->as<base::RealVectorStateSpace::StateType>()->values[4]);
            printf("-tempState vy: %f\n", tempState->as<base::RealVectorStateSpace::StateType>()->values[5]);
            printf("-tempState vz: %f\n", tempState->as<base::RealVectorStateSpace::StateType>()->values[6]);
            printf("-tempState vyaw: %f\n", tempState->as<base::RealVectorStateSpace::StateType>()->values[7]);
            printf("-sampleSteps: %d\n", sampleSteps);
            printf("-StepSize: %f\n", si_->getPropagationStepSize());*/
            //si_->printSettings();

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


    /*printf("------------------------------------------\n");
    printf("Salida PL: control t1: %f\n", control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0]);
    printf("Salida PL: control t2: %f\n", control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1]);
    printf("Salida PL: control t3: %f\n", control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[2]);
    printf("Salida PL: dest x: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[0]);
    printf("Salida PL: dest y: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[1]);
    printf("Salida PL: dest z: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[2]);
    printf("Salida PL: dest yaw: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[3]);
    printf("Salida PL: dest vx: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[4]);
    printf("Salida PL: dest vy: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[5]);
    printf("Salida PL: dest vz: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[6]);
    printf("Salida PL: dest vyaw: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[7]);
    printf("Salida PL: steps: %d\n", steps);*/

    return steps;
}
