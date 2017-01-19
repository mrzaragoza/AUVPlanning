#include "robots/AUVRobot.h"

using namespace ompl;

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace oauv = ompl::auvplanning;

ompl::auvplanning::AUVRobot::AUVRobot(YAML::Node config, int typeOfStateSpace) :
    sinf_(new oc::SpaceInformation(constructStateSpace(typeOfStateSpace),constructControlSpace(typeOfStateSpace))),
    ss_(sinf_),
    rbg_(),
    /*validitySvc_(new oauv::FCLStateValidityChecker(sinf_, rbg_.getGeometrySpecification(), getGeometricStateExtractor(), false)),*/
    dynamics_(new oauv::AUVDynamics()),
    odeSolver_(new oauv::ODEBasicSolver<>(sinf_, boost::bind(&oauv::AUVDynamics::ode, dynamics_, _1, _2, _3))),
    geometricStateSpace(new ob::RealVectorStateSpace(3)),
    config_(config)
{
    name_ = std::string("Torpedo shape robot");

    ss_.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver_, boost::bind(&oauv::AUVDynamics::postPropagate, dynamics_, _1, _2, _3, _4)));
}

void ompl::auvplanning::AUVRobot::setup(int type)
{
    OMPL_DEBUG("Robot Setup");

    setDefaultControlBounds();
    oauv::InferEnvironmentBounds(getGeometricComponentStateSpace(), rbg_);

    if (ss_.getProblemDefinition()->getStartStateCount() == 0)
    {
        OMPL_INFORM("Adding default start state");
        ss_.addStartState(getDefaultStartState());
    }

    oauv::InferProblemDefinitionBounds(ss_.getProblemDefinition(), getGeometricStateExtractor(), rbg_.getBoundsFactor(), rbg_.getBoundsAddition(),
                                             1, getGeometricComponentStateSpace());

    ompl::base::RealVectorBounds bounds = ss_.getStateSpace()->as<ob::RealVectorStateSpace>()->getBounds();
    ompl::base::RealVectorBounds boundsGeometric = getGeometricComponentStateSpace()->as<ob::RealVectorStateSpace>()->getBounds();

    bounds.setLow(0,boundsGeometric.low[0]);
    bounds.setLow(1,boundsGeometric.low[1]);
    bounds.setLow(2,boundsGeometric.low[2]);
    bounds.setLow(3,-M_PI);
    bounds.setLow(4,-2);
    bounds.setLow(5,-2);
    bounds.setLow(6,-2);
    bounds.setLow(7,-2);
    bounds.setHigh(0,boundsGeometric.high[0]);
    bounds.setHigh(1,boundsGeometric.high[1]);
    bounds.setHigh(2,boundsGeometric.high[2]);
    bounds.setHigh(3,M_PI);
    bounds.setHigh(4,2);
    bounds.setHigh(5,2);
    bounds.setHigh(6,2);
    bounds.setHigh(7,2);

    ss_.getStateSpace()->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    validitySvc_ = rbg_.allocStateValidityChecker(sinf_,  getGeometricStateExtractor(), false);

    if (ss_.getStateValidityChecker() != validitySvc_){
        //OMPL_DEBUG("AUVRobot::setup ss_.getStateValidityChecker() es distinto de validitySvc_. \n\tSe hace un set para cambiarlo");
        ss_.setStateValidityChecker(validitySvc_);
    }

    ss_.getStateSpace()->setup();

    if (!ss_.getStateSpace()->hasDefaultProjection()){
        //OMPL_DEBUG("GeometricStateProjector a realizar");
        ss_.getStateSpace()->registerDefaultProjection(auvplanning::allocGeometricStateProjector(ss_.getStateSpace(),
                                                                   getGeometricComponentStateSpace(),
                                                                   getGeometricStateExtractor()));
        //OMPL_DEBUG("GeometricStateProjector realizado");
    }

    ss_.setup();

    setDirectedControlSampler(type);

}

ob::ScopedState<> ompl::auvplanning::AUVRobot::getFullStateFromGeometricComponent(const ob::ScopedState<> &state) const
{
    ob::ScopedState<> s(ss_.getStateSpace());
    std::vector <double> reals = state.reals ();

    s = 0.0;
    for (size_t i = 0; i < reals.size (); ++i)
        s[i] = reals[i];
    return s;
    //return state;
}


ob::ScopedState<> ompl::auvplanning::AUVRobot::getDefaultStartState(void) const
{
    ob::ScopedState<ob::RealVectorStateSpace> s(getGeometricComponentStateSpace());

    s[0] = 0.;
    s[1] = 0.;
    s[2] = 0.;
    return getFullStateFromGeometricComponent(s);
}

//Las acciones de control en los trhusters van de -1 a 1. 
//Siendo 1 el par de fuerza máximo del thruster en el sentido de avance, y -1 el máximo en el contrario.
void ompl::auvplanning::AUVRobot::setDefaultControlBounds(void)
{
    ob::RealVectorBounds cbounds(3);
    cbounds.setLow(-1.);
    cbounds.setHigh(1.);
    ss_.getControlSpace()->as<oc::RealVectorControlSpace>()->setBounds(cbounds);
}

ompl::control::DirectedControlSamplerPtr ompl::auvplanning::AUVRobot::AUVSemiRandomDirectedControlSamplerAllocator(const ompl::control::SpaceInformation *si, unsigned int k, YAML::Node config)
{
    return ompl::control::DirectedControlSamplerPtr(new oauv::AUVSemiRandomDirectedControlSampler(si, k, config));
}

ompl::control::DirectedControlSamplerPtr ompl::auvplanning::AUVRobot::AUV2StepPIDControlSamplerAllocator(const ompl::control::SpaceInformation *si, YAML::Node config)
{
    return ompl::control::DirectedControlSamplerPtr(new oauv::AUV2StepPIDControlSampler(si, config));
}

ompl::control::DirectedControlSamplerPtr ompl::auvplanning::AUVRobot::AUVPIDControlSamplerAllocator(const ompl::control::SpaceInformation *si, unsigned int k, YAML::Node config)
{
    return ompl::control::DirectedControlSamplerPtr(new oauv::AUVPIDControlSampler(si, k, config));
}

ompl::control::DirectedControlSamplerPtr ompl::auvplanning::AUVRobot::RandomDirectedControlSamplerAllocator(const ompl::control::SpaceInformation *si, unsigned int k)
{
    return ompl::control::DirectedControlSamplerPtr(new oc::SimpleDirectedControlSampler(si, k));
}

void ompl::auvplanning::AUVRobot::setDirectedControlSampler(int type){

    control::DirectedControlSamplerAllocator dcsa;

    switch(type){
        case AUV_SIMPLE_DCS:
            dcsa = boost::bind(RandomDirectedControlSamplerAllocator, sinf_.get(), 15);
            break;
        case AUV_SEMI_RANDOM_DCS:
            dcsa = boost::bind(AUVSemiRandomDirectedControlSamplerAllocator, sinf_.get(), 15, config_);
            break;
        case AUV_PID_DCS:
            dcsa = boost::bind(AUVPIDControlSamplerAllocator, sinf_.get(), 15, config_);
            break;
        case AUV_2PID_DCS:
            dcsa = boost::bind(AUV2StepPIDControlSamplerAllocator, sinf_.get(), config_);
            break;
        default:
            dcsa = boost::bind(RandomDirectedControlSamplerAllocator, sinf_.get(), 15);
    }

    sinf_.get()->setDirectedControlSamplerAllocator(dcsa);

}
