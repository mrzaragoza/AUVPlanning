#include "robots/control/AUVPID.h"
#include <ompl/control/ODESolver.h>
#include "robots/dynamics/AUVdynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

ompl::controller::AUVPID::AUVPID(const control::SpaceInformation *si) :
Controller(si),
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

}

unsigned int ompl::controller::AUVPID::propagation(const base::State *source, base::State *dest, double steps, bool checkValidity)
{
    const unsigned int minDuration = si_->getMinControlDuration();

    ompl::control::RealVectorControlSpace::ControlType *newControl = sinf->allocControl();

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
    while(tiempo <= minDuration && (abs(dist_actual) > rango_dist_objetivo || abs(dist_z) > rango_profundidad_objetivo)){

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
	
	si_->copyState(dest, stateTemp);
    si_->freeState(stateTemp);
    si_->freeState(stateRes);
    return tiempo;
}


double ompl::controller::AUVPID::pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw){

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