#include "robots/control/AUVPID.h"
#include <ompl/control/ODESolver.h>
#include "robots/dynamics/AUVdynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

ompl::controller::AUVPID::AUVPID(const control::SpaceInformation *si) : Controller(si)
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

unsigned int ompl::controller::AUVPID::propagation(const base::State *source, base::State *dest, unsigned int steps, bool checkValidity)
{
    const unsigned int maxDuration = sinf->getMaxControlDuration();

    control::Control *newControl = sinf->allocControl();

    base::State         *stateTemp = sinf->allocState();
    base::State         *stateRes = sinf->allocState();
    sinf->copyState(stateTemp,source);	

    double 				z = source->as<base::RealVectorStateSpace::StateType>()->values[2];
    double 				yaw = source->as<base::RealVectorStateSpace::StateType>()->values[3];

    double				pre_errorz = 0;
    double				pre_errorsurge = 0;
    double				pre_erroryaw = 0;
    double				integralz = 0;
    double				integralsurge = 0;
    double				integralyaw = 0;

    double 				fSurge = 0, fZ = 0, mZ = 0;
    double 				tau0 = 0, tau1 = 0, tau2 = 0;
    unsigned int 		tiempo = 0;

    double 				dist_x = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
	double 				dist_y = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];
	double 				dist_z = dest->as<base::RealVectorStateSpace::StateType>()->values[2] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[2];
    
    double 				z_ref = dest->as<base::RealVectorStateSpace::StateType>()->values[2];
	double				heading = atan2(dist_y,dist_x); //radianes
	double 				dist_inicial = sqrt(pow(dist_y,2) + pow(dist_x,2));
	double 				dist_actual = sqrt(pow(dist_y,2) + pow(dist_x,2));

	rango_dist_objetivo = 1.5;
    rango_profundidad_objetivo = 0.5;
    
    while(tiempo <= maxDuration && (fabs(dist_actual) > rango_dist_objetivo || fabs(dist_z) > rango_profundidad_objetivo)){

	    fZ = pid(z_ref, z, stepSize, Kpz, Kdz, Kiz, &pre_errorz, &integralz, false);
	    mZ = pid(heading,yaw, stepSize, Kpyaw, Kdyaw, Kiyaw, &pre_erroryaw, &integralyaw, true);
	    fSurge = pid(dist_inicial, dist_inicial-dist_actual, stepSize, Kpsurge, Kdsurge, Kisurge, &pre_errorsurge, &integralsurge, false);

	    //Traducción de las fuerzas y mometos a las acciones de control
	    tau0 = (fZ/max_fuerza_motores) + controlZEstable;
	    tau1 = (fSurge + mZ/l_motores)/(2*max_fuerza_motores);
	    tau2 = (fSurge - mZ/l_motores)/(2*max_fuerza_motores);

	    if(tau0 > 1)  tau0 = 1;
	    else if(tau0 < -1) tau0 = -1;
	    if(tau1 > 1)  tau1 = 1;
	    else if(tau1 < -1) tau1 = -1;
	    if(tau2 > 1)  tau2 = 1;
	    else if(tau2 < -1) tau2 = -1;
/*
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

	    }*/
	    
	    //printf("Control: %f %f %f\n",tau0, tau1, tau2);

	    newControl->as<control::RealVectorControlSpace::ControlType>()->values[0] = tau0;
	    newControl->as<control::RealVectorControlSpace::ControlType>()->values[1] = tau1;
	    newControl->as<control::RealVectorControlSpace::ControlType>()->values[2] = tau2;	
	
    	stPropagator->propagate(stateTemp,newControl,stepSize,stateRes);

    	sinf->copyState(stateTemp,stateRes);

    	printf("%f %f %f %f %f %f %f %f", stateRes->as<base::RealVectorStateSpace::StateType>()->values[0],
    	stateRes->as<base::RealVectorStateSpace::StateType>()->values[1],
    	stateRes->as<base::RealVectorStateSpace::StateType>()->values[2],
    	stateRes->as<base::RealVectorStateSpace::StateType>()->values[3],
    	stateRes->as<base::RealVectorStateSpace::StateType>()->values[4],
    	stateRes->as<base::RealVectorStateSpace::StateType>()->values[5],
    	stateRes->as<base::RealVectorStateSpace::StateType>()->values[6],
    	stateRes->as<base::RealVectorStateSpace::StateType>()->values[7]);

    	printf("\r");

    	dist_x = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
	    dist_y = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - 
	                                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];
		dist_z = z_ref - stateTemp->as<base::RealVectorStateSpace::StateType>()->values[2];

	    dist_actual = sqrt(pow(dist_x,2) + pow(dist_y,2));
    	heading = atan2(dist_y,dist_x);
	    //printf("Tiempo: %f dist_actual: %f dist_z: %f\n",tiempo*stepSize, dist_actual, dist_z);

    	z = stateTemp->as<base::RealVectorStateSpace::StateType>()->values[2];
		yaw = stateTemp->as<base::RealVectorStateSpace::StateType>()->values[3];

	    tiempo++;
	}
	
	sinf->copyState(dest, stateTemp);
    sinf->freeState(stateTemp);
    sinf->freeState(stateRes);
    sinf->freeControl(newControl);

    printf("\n[AUVPID] result x: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[0]);
	printf("[AUVPID] result y: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[1]);
	printf("[AUVPID] result z: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[2]);
	printf("[AUVPID] result yaw: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[3]);
	printf("[AUVPID] result vx: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[4]);
	printf("[AUVPID] result vy: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[5]);
	printf("[AUVPID] result vz: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[6]);
	printf("[AUVPID] result vyaw: %f\n", dest->as<base::RealVectorStateSpace::StateType>()->values[7]);
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
    /*printf("error = %f\n", error);
    printf("pre error = %f\n", *pre_error);
    printf("integral = %f\n", *integral);*/
    double output = p + i + d;

    if(output > 5) output = 5;
    else if(output < -5) output = -5;

    *pre_error = error;

    return output;

}
