#include "planners/PlanificadorLocal/AUVPIDControlSampler.h"
#include <ompl/control/ODESolver.h>
#include "robots/dynamics/AUVdynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace oauv = ompl::auvplanning;

ompl::auvplanning::AUVPIDControlSampler::AUVPIDControlSampler(const control::SpaceInformation *si, unsigned int k, YAML::Node config) :
control::DirectedControlSampler(si), numControlSamples_(k),
sinf(si),
dynamics_(new oauv::AUVDynamics()),
ode_(new ompl::control::ODEBasicSolver<>(sinf, boost::bind(&oauv::AUVDynamics::ode, dynamics_, _1, _2, _3))),
inicial(si->allocState()),
reference(si->allocState())
{
    YAML::Node robot_config = YAML::LoadFile("../includes/robots/torpedo.yaml");

    Kpz = robot_config["torpedo/Kz"][0].as<double>();
    Kdz = robot_config["torpedo/Kz"][1].as<double>();
    Kiz = robot_config["torpedo/Kz"][2].as<double>();
    Kpsurge = robot_config["torpedo/Ksurge"][0].as<double>();
    Kdsurge = robot_config["torpedo/Ksurge"][1].as<double>();
    Kisurge = robot_config["torpedo/Ksurge"][2].as<double>();
    Kpyaw = robot_config["torpedo/Kyaw"][0].as<double>();
    Kdyaw = robot_config["torpedo/Kyaw"][1].as<double>();
    Kiyaw = robot_config["torpedo/Kyaw"][2].as<double>();    

    mass = robot_config["torpedo/mass"].as<double>();
    c_rbm1 = robot_config["torpedo/rbMassCoefficients"][0].as<double>();
    c_rbm3 = robot_config["torpedo/rbMassCoefficients"][2].as<double>();
    c_rbm4 = robot_config["torpedo/rbMassCoefficients"][3].as<double>();
    c_am1 = robot_config["torpedo/aMassCoefficients"][0].as<double>();
    c_am3 = robot_config["torpedo/aMassCoefficients"][2].as<double>();
    c_am4 = robot_config["torpedo/aMassCoefficients"][3].as<double>();
    c_ld1 = robot_config["torpedo/dampingCoefficients"][0].as<double>();
    c_ld3 = robot_config["torpedo/dampingCoefficients"][2].as<double>();
    c_ld4 = robot_config["torpedo/dampingCoefficients"][3].as<double>();
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

    stepSize = si->getPropagationStepSize();
    stPropagator = oc::ODESolver::getStatePropagator(ode_, boost::bind(&oauv::AUVDynamics::postPropagate, dynamics_, _1, _2, _3, _4));

}

ompl::auvplanning::AUVPIDControlSampler::~AUVPIDControlSampler()
{
	si_->freeState(reference);
	si_->freeState(inicial);
}

unsigned int ompl::auvplanning::AUVPIDControlSampler::sampleTo(control::Control *control, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

unsigned int ompl::auvplanning::AUVPIDControlSampler::sampleTo(control::Control *control, const control::Control *previous, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

unsigned int ompl::auvplanning::AUVPIDControlSampler::getBestControl (control::Control *control, const base::State *source, base::State *dest)
{
    isPIDResetNeeded(source,dest);
    const unsigned int minDuration = si_->getMinControlDuration();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::RealVectorControlSpace::ControlType*>(control);

    base::State         *stateTemp = si_->allocState();
    base::State         *stateRes = si_->allocState();
    si_->copyState(stateTemp,source);	

    double z_ref = dest->as<base::RealVectorStateSpace::StateType>()->values[2];
    double z = source->as<base::RealVectorStateSpace::StateType>()->values[2];
    double yaw = source->as<base::RealVectorStateSpace::StateType>()->values[3];

    //variables de control
    double fSurge = 0, fZ = 0, mZ = 0;
    double dt = si_->getPropagationStepSize();
    double tau0 = 0, tau1 = 0, tau2 = 0;
    unsigned int tiempo = 0.0;

    double dist_x = reference->as<base::RealVectorStateSpace::StateType>()->values[0] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
	double dist_y = reference->as<base::RealVectorStateSpace::StateType>()->values[1] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];
	double dist_z = reference->as<base::RealVectorStateSpace::StateType>()->values[2] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[2];

	double dist_actual = sqrt(pow(dist_x,2) + pow(dist_y,2));

	/*printf("Source: %f %f %f %f %f %f %f %f\n", source->as<base::RealVectorStateSpace::StateType>()->values[0],
			source->as<base::RealVectorStateSpace::StateType>()->values[1],source->as<base::RealVectorStateSpace::StateType>()->values[2],
			source->as<base::RealVectorStateSpace::StateType>()->values[3],source->as<base::RealVectorStateSpace::StateType>()->values[4],
			source->as<base::RealVectorStateSpace::StateType>()->values[5],source->as<base::RealVectorStateSpace::StateType>()->values[6],
			source->as<base::RealVectorStateSpace::StateType>()->values[7]);

	printf("Reference: %f %f %f %f %f %f %f %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[0],
			dest->as<base::RealVectorStateSpace::StateType>()->values[1],dest->as<base::RealVectorStateSpace::StateType>()->values[2],
			dest->as<base::RealVectorStateSpace::StateType>()->values[3],dest->as<base::RealVectorStateSpace::StateType>()->values[4],
			dest->as<base::RealVectorStateSpace::StateType>()->values[5],dest->as<base::RealVectorStateSpace::StateType>()->values[6],
			dest->as<base::RealVectorStateSpace::StateType>()->values[7]);*/

	//printf("getBestControl. Pre bucle\n");
	//std::cout << std::boolalpha << (tiempo <= minDuration && (abs(dist_actual) > rango_dist_objetivo || abs(dist_z) > rango_profundidad_objetivo)) << std::endl;
    while(tiempo <= minDuration && (abs(dist_actual) > rango_dist_objetivo || abs(dist_z) > rango_profundidad_objetivo)){

    	//printf("getBestControl. en bucle");

	    fZ = pid(z_ref, z, dt, Kpz, Kdz, Kiz, &pre_errorz, &integralz, false);
	    mZ = pid(heading, yaw, dt, Kpyaw, Kdyaw, Kiyaw, &pre_erroryaw, &integralyaw, true);
	    fSurge = pid(0, dist_inicial, dt, Kpsurge, Kdsurge, Kisurge, &pre_errorsurge, &integralsurge, false);

	    //Traducción de las fuerzas y mometos a las acciones de control
	    tau0 = (fZ/max_fuerza_motores) + controlZEstable;
	    tau1 = fSurge/2 - mZ/(2*l_motores);
	    tau2 = fSurge/2 + mZ/(2*l_motores);

	    tau1 = tau1/max_fuerza_motores;
	    tau2 = tau2/max_fuerza_motores;

	    if(tau0 > 1)  tau0 = 1;
	    else if(tau0 < -1) tau0 = -1;

	    if ( tau1 > 1 || tau2 > 1 ){

	        if( tau1 > tau2 ){
	            tau2 = tau2/tau1;
	            tau1 = 1;
	        }else{
	            tau1 = tau1/tau2;
	            tau2 = 1;
	        }   
	    }

	    if ( tau1 < -1 || tau2 < -1 ){

	        if( tau1 < tau2 ){
	            tau2 = -tau2/tau1;
	            tau1 = -1;
	        }else{
	            tau1 = -tau1/tau2;
	            tau2 = -1;
	        }

	    }

	    newControl->as<control::RealVectorControlSpace::ControlType>()->values[0] = tau0;
	    newControl->as<control::RealVectorControlSpace::ControlType>()->values[1] = tau1;
	    newControl->as<control::RealVectorControlSpace::ControlType>()->values[2] = tau2;	
	
    	stPropagator->propagate(stateTemp,newControl,stepSize,stateRes);

    	/*printf("getBestControl. Estado -> %f, %f, %f, %f\n", stateRes->as<base::RealVectorStateSpace::StateType>()->values[0],
			stateRes->as<base::RealVectorStateSpace::StateType>()->values[1],stateRes->as<base::RealVectorStateSpace::StateType>()->values[2],
			stateRes->as<base::RealVectorStateSpace::StateType>()->values[3]);
		printf("Control %f, %f, %f\n", tau0,tau1,tau2);*/

    	/*if(!si_->isValid(stateRes)){
    		//printf("getBestControl. NO VALIDO!!\n");
    		break;
    	}*/
    	si_->copyState(stateTemp,stateRes);
    	tiempo + 1;

    	dist_x = reference->as<base::RealVectorStateSpace::StateType>()->values[0] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
	    dist_y = reference->as<base::RealVectorStateSpace::StateType>()->values[1] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];
		dist_z = reference->as<base::RealVectorStateSpace::StateType>()->values[2] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[2];

	    dist_actual = sqrt(pow(dist_x,2) + pow(dist_y,2));
	    tiempo++;
	}
	//printf("\n");
	
	si_->copyState(dest, stateTemp);
    si_->freeState(stateTemp);
    si_->freeState(stateRes);
	/*printf("%f %f %f %f %f %f %f %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[0],
		dest->as<base::RealVectorStateSpace::StateType>()->values[1],dest->as<base::RealVectorStateSpace::StateType>()->values[2],
		dest->as<base::RealVectorStateSpace::StateType>()->values[3],dest->as<base::RealVectorStateSpace::StateType>()->values[4],
		dest->as<base::RealVectorStateSpace::StateType>()->values[5],dest->as<base::RealVectorStateSpace::StateType>()->values[6],
		dest->as<base::RealVectorStateSpace::StateType>()->values[7]);*/
    return tiempo;
}


double ompl::auvplanning::AUVPIDControlSampler::pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw){

    double error = reference - value;

    if (isYaw && fabs(error) > M_PI){
        value = - value;
        error = reference - value;
    }

    double p = Kp * error;

    *integral = *integral + error*dt;
    double i = Ki * *integral;

    double d = Kd * (error - *pre_error)/dt;

    double output = p + i + d;

    if(output > 1) output = 1;
    else if(output < -1) output = -1;

    *pre_error = error;

    return output;

}

void ompl::auvplanning::AUVPIDControlSampler::isPIDResetNeeded(const base::State *init, const base::State *dest){

	if(si_->equalStates(reference,dest) == false){
		si_->copyState(inicial, init);
		si_->copyState(reference, dest);

	    //Distancias con estado inicial
	    dist_x_inicial       = reference->as<base::RealVectorStateSpace::StateType>()->values[0] - 
	                                        inicial->as<base::RealVectorStateSpace::StateType>()->values[0];
	    dist_y_inicial       = reference->as<base::RealVectorStateSpace::StateType>()->values[1] - 
	                                        inicial->as<base::RealVectorStateSpace::StateType>()->values[1];

	    dist_inicial         = sqrt(pow(dist_x_inicial,2) + pow(dist_y_inicial,2));
		
	    //Calculo del ángulo que hay entre el estado inicial y el final
	    heading = atan2(dist_y_inicial,dist_x_inicial); //radianes

	    //Cálculo del rango de aceptación en el que se considera que se ha llegado al objetivo
	    rango_dist_objetivo = dist_inicial * porcentaje_dist_rango_objetivo;
	    rango_profundidad_objetivo = dist_z_inicial * porcentaje_dist_profundidad_rango_objetivo;

	    if(rango_dist_objetivo < rango_min_objetivo) rango_dist_objetivo = rango_min_objetivo;
	    else if(rango_dist_objetivo > rango_max_objetivo) rango_dist_objetivo = rango_max_objetivo;

	    if(rango_profundidad_objetivo < rango_profundidad_min_objetivo) rango_profundidad_objetivo = rango_profundidad_min_objetivo;
	    else if(rango_profundidad_objetivo > rango_profundidad_max_objetivo) rango_profundidad_objetivo = rango_profundidad_max_objetivo;

		pre_errorz = 0;
		pre_errorsurge = 0;
		pre_erroryaw = 0;
		integralz = 0;
		integralsurge = 0;
		integralyaw = 0;
		//printf("Reset done\n");
	}
}