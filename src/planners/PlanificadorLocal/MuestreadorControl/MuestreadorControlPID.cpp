#include "planners/PlanificadorLocal/MuestreadorControl/MuestreadorControlPID.h"

ompl::base::State  *ompl::auvplanning::MuestreadorControlPID::goal_ = NULL;

ompl::auvplanning::MuestreadorControlPID::MuestreadorControlPID(const ompl::control::ControlSpace *space,YAML::Node config = YAML::LoadFile("test.yaml");) : control::ControlSampler::ControlSampler(space),space_mc(space)
{

	YAML::Node robot_config = YAML::LoadFile("/includes/robots/torpedo.yaml");

	Kpz = robot_config["torpedo/Kz"][0].as<double>();
	Kdz = robot_config["torpedo/Kz"][1].as<double>();
	Kiz = robot_config["torpedo/Kz"][2].as<double>();
    Kpsurge = robot_config["torpedo/Ksurge"][0].as<double>();
    Kdsurge = robot_config["torpedo/Ksurge"][1].as<double>();
    Kisurge = robot_config["torpedo/Ksurge"][2].as<double>();
    Kpyaw = robot_config["torpedo/Kyaw"][0].as<double>();
    Kdyaw = robot_config["torpedo/Kyaw"][1].as<double>();
    Kiyaw = robot_config["torpedo/Kyaw"][2].as<double>();
    pre_errorz = 0;
    pre_errorsurge = 0;
    pre_erroryaw = 0;
    integralz = 0;
    integralsurge = 0;
    integralyaw = 0;

    mass = robot_config["torpedo/mass"].as<double>();
    c_rbm1 = robot_config["torpedo/rbMassCoefficients"][0].as<double>();
    c_rbm3 = robot_config["torpedo/rbMassCoefficients"][2].as<double>();
    c_rbm4 = robot_config["torpedo/rbMassCoefficients"][3].as<double>();
    c_am1 = robot_config["torpedo/aMassCoefficients"][0].as<double>();
    c_am3 = robot_config["torpedo/aMassCoefficients"][2].as<double>();
    c_am4 = robot_config["torpedo/aMassCoefficients"][3].as<double>();
    c_ld1 = robot_config["torpedo/dampingCoefficientss"][0].as<double>();
    c_ld3 = robot_config["torpedo/dampingCoefficientss"][2].as<double>();
    c_ld4 = robot_config["torpedo/dampingCoefficientss"][3].as<double>();
    c_qd1 = robot_config["torpedo/quadraticDampingCoefficients"][0].as<double>();
    c_qd3 = robot_config["torpedo/quadraticDampingCoefficients"][2].as<double>();
    c_qd4 = robot_config["torpedo/quadraticDampingCoefficients"][3].as<double>();
    controlZEstable = robot_config["torpedo/controlZEstable"].as<double>();
    l_motores = robot_config["torpedo/lengths"][2].as<double>();
    max_fuerza_motores = robot_config["torpedo/max_fuerza_motores"].as<double>();

    porcentaje_dist_rango_objetivo = robot_config["control/porcentaje_dist_rango_objetivo"].as<double>();
    rango_min_objetivo = robot_config["control/rango_min_objetivo"].as<double>();
    rango_max_objetivo = robot_config["control/rango_max_objetivo"].as<double>();
    porcentaje_dist_profundidad_rango_objetivo = robot_config["control/porcentaje_dist_profundidad_rango_objetivo"].as<double>();
    rango_profundidad_min_objetivo = robot_config["control/rango_profundidad_min_objetivo"].as<double>();
    rango_profundidad_max_objetivo = robot_config["control/rango_profundidad_max_objetivo"].as<double>();

}

void ompl::auvplanning::MuestreadorControlPID::sample(control::Control *control)
{
    const unsigned int dim = space_mc->getDimension();

    const base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(space_mc)->getBounds();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::RealVectorControlSpace::ControlType*>(control);

    for (unsigned int i = 0 ; i < dim ; ++i)
        //newControl->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);/////////////////
        newControl->values[i] = ((double) std::rand() / RAND_MAX)*2-1;/////////////////
}

void ompl::auvplanning::MuestreadorControlPID::sample(control::Control *control, const base::State * state)
{

    const unsigned int dim = space_mc->getDimension();
    if (dim == 8){
    	twoStepsPID(control, state);
    }
		
}

void ompl::auvplanning::MuestreadorControlPID::sampleNext(control::Control *control, const control::Control * /* previous */)
{
    sample(control);
}

void ompl::auvplanning::MuestreadorControlPID::sampleNext(control::Control *control, const control::Control * /* previous */, const base::State * state)
{
    sample(control,state);
}

unsigned int ompl::auvplanning::MuestreadorControlPID::sampleStepCount(unsigned int minSteps, unsigned int maxSteps)
{
    //return rng_.uniformInt(minSteps, maxSteps);/////////////////
    return  std::rand() %(maxSteps-minSteps+1)+minSteps;/////////////////
    //return  (maxSteps+minSteps)/2;/////////////////
}

void ompl::auvplanning::MuestreadorControlPID::setGoal(base::State *state)
{
    goal_ = state;
}

ompl::base::State* ompl::auvplanning::MuestreadorControlPID::getGoal()
{
    return goal_;
}

void ompl::auvplanning::MuestreadorControlPID::twoStepsPID(control::Control *control, const base::State *state){

    //const base::RealVectorBounds &bounds = static_cast<const ompl::control::CompoundControlSpace*>(space_mc)->getBounds();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::CompoundControlSpace::ControlType*>(control);

	//Distancias con estado actual
	double diff_x 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[0] - 
										state->as<base::RealVectorStateSpace::StateType>()->values[0];
	double diff_y 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[1] - 
										state->as<base::RealVectorStateSpace::StateType>()->values[1];
	double dist_z_inicial		= goal_->as<base::RealVectorStateSpace::StateType>()->values[2] - 
										state->as<base::RealVectorStateSpace::StateType>()->values[2];

	double dist_inicial			= sqrt(pow(diff_x,2) + pow(diff_y,2));

	//Cálculo del rango de aceptación en el que se considera que se ha llegado al objetivo
	double rango_dist_objetivo = dist_inicial * porcentaje_dist_rango_objetivo;
	double rango_profundidad_objetivo = dist_z_inicial * porcentaje_dist_profundidad_rango_objetivo;

	if(rango_dist_objetivo < rango_min_objetivo) rango_dist_objetivo = rango_min_objetivo;
	else if(rango_dist_objetivo > rango_max_objetivo) rango_dist_objetivo = rango_max_objetivo;

	if(rango_profundidad_objetivo < rango_profundidad_min_objetivo) rango_profundidad_objetivo = rango_profundidad_min_objetivo;
	else if(rango_profundidad_objetivo > rango_profundidad_max_objetivo) rango_profundidad_objetivo = rango_profundidad_max_objetivo;

	//Calculo del ángulo que hay entre el estado actual y el final y la diferencia entre éste y el yaw del vehículo
	double heading = atan2(diff_y,diff_x); //radianes
	double yaw = state->as<base::RealVectorStateSpace::StateType>()->values[3];

	base::State *stateTemp;
    stateGoal = si->allocState();
    stateGoal->as<base::RealVectorStateSpace::StateType>()->values[0] = config["general/goal"][0].as<double>();

	while(){
		heading = atan2(diff_y,diff_x); //radianes		
		yaw = state->as<base::RealVectorStateSpace::StateType>()->values[3];
	}

	while(){

	}
}