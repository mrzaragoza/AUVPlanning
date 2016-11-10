#include "planners/PlanificadorLocal/AUVDirectedControlSampler.h"
#include <ompl/control/ODESolver.h>
#include "robots/dynamics/AUVdynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace oauv = ompl::auvplanning;

ompl::auvplanning::AUVDirectedControlSampler::AUVDirectedControlSampler(const control::SpaceInformation *si, unsigned int k, YAML::Node config) :
control::DirectedControlSampler(si), numControlSamples_(k),
sinf(si),
dynamics_(new oauv::AUVDynamics()),
ode_(new ompl::control::ODEBasicSolver<>(sinf, boost::bind(&oauv::AUVDynamics::ode, dynamics_, _1, _2, _3))),
cSpace_(new control::RealVectorControlSpace(si->getStateSpace(), 3))
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

ompl::auvplanning::AUVDirectedControlSampler::~AUVDirectedControlSampler()
{
}

unsigned int ompl::auvplanning::AUVDirectedControlSampler::sampleTo(control::Control *control, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

unsigned int ompl::auvplanning::AUVDirectedControlSampler::sampleTo(control::Control *control, const control::Control *previous, const base::State *source, base::State *dest)
{
    return getBestControl(control, source, dest);
}

unsigned int ompl::auvplanning::AUVDirectedControlSampler::getBestControl (control::Control *control, const base::State *source, base::State *dest)
{
    
    const unsigned int minDuration = si_->getMinControlDuration();
    const unsigned int maxDuration = si_->getMaxControlDuration();

    //const base::RealVectorBounds &bounds = static_cast<const ompl::control::CompoundControlSpace*>(space_mc)->getBounds();

    ompl::control::CompoundControlSpace::ControlType *newControl = static_cast<ompl::control::CompoundControlSpace::ControlType*>(control);

    //Distancias con estado actual
    double dist_x_inicial       = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - 
                                        source->as<base::RealVectorStateSpace::StateType>()->values[0];
    double dist_y_inicial       = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - 
                                        source->as<base::RealVectorStateSpace::StateType>()->values[1];
    double dist_z_inicial       = dest->as<base::RealVectorStateSpace::StateType>()->values[2] - 
                                        source->as<base::RealVectorStateSpace::StateType>()->values[2];

    double dist_inicial         = sqrt(pow(dist_x_inicial,2) + pow(dist_y_inicial,2));

    //Cálculo del rango de aceptación en el que se considera que se ha llegado al objetivo
    double rango_dist_objetivo = dist_inicial * porcentaje_dist_rango_objetivo;
    double rango_profundidad_objetivo = dist_z_inicial * porcentaje_dist_profundidad_rango_objetivo;

    if(rango_dist_objetivo < rango_min_objetivo) rango_dist_objetivo = rango_min_objetivo;
    else if(rango_dist_objetivo > rango_max_objetivo) rango_dist_objetivo = rango_max_objetivo;

    if(rango_profundidad_objetivo < rango_profundidad_min_objetivo) rango_profundidad_objetivo = rango_profundidad_min_objetivo;
    else if(rango_profundidad_objetivo > rango_profundidad_max_objetivo) rango_profundidad_objetivo = rango_profundidad_max_objetivo;

    //Calculo del ángulo que hay entre el estado actual y el final
    double heading = atan2(dist_y_inicial,dist_x_inicial); //radianes
    double yaw = source->as<base::RealVectorStateSpace::StateType>()->values[3];
    double z = source->as<base::RealVectorStateSpace::StateType>()->values[2];
    double z_ref = dest->as<base::RealVectorStateSpace::StateType>()->values[2];

    base::State         *stateTemp = si_->allocState();
    base::State         *stateRes = si_->allocState();
    si_->copyState(stateTemp,source);
    control::Control    *tempControl = cSpace_->allocControl();

    //variables de control
    double fSurge = 0, fZ = 0, mZ = 0;
    double dt = si_->getPropagationStepSize();
    double tau0 = 0, tau1 = 0, tau2 = 0, sumTau0 = 0, sumTau1 = 0, sumTau2 = 0;
    double tiempo = 0.0;
    double pre_errorz = 0;
    double pre_errorsurge = 0;
    double pre_erroryaw = 0;
    double integralz = 0;
    double integralsurge = 0;
    double integralyaw = 0;

    bool haLlegado = false;

    double dist_x = 0; 
    double dist_y = 0; 

    //Bucle para la z y el yaw
    while(!haLlegado){
        //Calculo del ángulo que hay entre el estado actual y el final
        dist_x  = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - 
                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
        dist_y  = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - 
                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];
        heading = atan2(dist_y,dist_x); //radianes      
        yaw = stateTemp->as<base::RealVectorStateSpace::StateType>()->values[3];
        z = stateTemp->as<base::RealVectorStateSpace::StateType>()->values[2];

        fZ = pid(z_ref, z, dt, Kpz, Kdz, Kiz, &pre_errorz, &integralz, false);
        mZ = pid(heading, yaw, dt, Kpyaw, Kdyaw, Kiyaw, &pre_erroryaw, &integralyaw, true);

        //Traducción de las fuerzas y mometos a las acciones de control
        tau0 = (fZ/max_fuerza_motores) + controlZEstable;
        tau1 = mZ/(2*l_motores);
        tau2 = -mZ/(2*l_motores);

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

        tempControl->as<control::RealVectorControlSpace::ControlType>()->values[0] = tau0;
        tempControl->as<control::RealVectorControlSpace::ControlType>()->values[1] = tau1;
        tempControl->as<control::RealVectorControlSpace::ControlType>()->values[2] = tau2;
        sumTau0 = sumTau0 + tau0;
        sumTau1 = sumTau1 + tau1;
        sumTau2 = sumTau2 + tau2;
        
        stPropagator->propagate(stateTemp,tempControl,stepSize,stateRes);
        si_->copyState(stateTemp,stateRes);
        tiempo = tiempo + stepSize;

        if( fabs(pre_erroryaw) < 0.1 && fabs(pre_errorz) < rango_profundidad_objetivo) haLlegado = true;

    }

    newControl->components[0]->as<control::RealVectorControlSpace::ControlType>()->values[0] = sumTau0 / tiempo;
    newControl->components[0]->as<control::RealVectorControlSpace::ControlType>()->values[1] = sumTau1 / tiempo;
    newControl->components[0]->as<control::RealVectorControlSpace::ControlType>()->values[2] = sumTau2 / tiempo;
    newControl->components[0]->as<control::RealVectorControlSpace::ControlType>()->values[3] = tiempo;

    int steps = (int) tiempo/stepSize;

    haLlegado = false;
    tiempo = 0.0;
    sumTau0 = 0;
    sumTau1 = 0;
    sumTau2 = 0;

    double dist_actual = 0;

    while(!haLlegado){

        //Cálculo de la distancia actual
        dist_x  = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - 
                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
        dist_y  = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - 
                        stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];

        dist_actual = sqrt(pow(dist_x,2) + pow(dist_y,2));

        fZ = pid(z_ref, z, dt, Kpz, Kdz, Kiz, &pre_errorz, &integralz, false);
        fSurge = pid(dist_inicial, dist_inicial-dist_actual, dt, Kpsurge, Kdsurge, Kisurge, &pre_errorsurge, &integralsurge, false);

        //Traducción de las fuerzas y mometos a las acciones de control
        tau0 = (fZ/max_fuerza_motores) + controlZEstable;
        tau1 = fSurge/2;
        tau2 = fSurge/2;

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

        tempControl->as<control::RealVectorControlSpace::ControlType>()->values[0] = tau0;
        tempControl->as<control::RealVectorControlSpace::ControlType>()->values[1] = tau1;
        tempControl->as<control::RealVectorControlSpace::ControlType>()->values[2] = tau2;
        sumTau0 = sumTau0 + tau0;
        sumTau1 = sumTau1 + tau1;
        sumTau2 = sumTau2 + tau2;
        
        stPropagator->propagate(stateTemp,tempControl,stepSize,stateRes);
        si_->copyState(stateTemp,stateRes);
        tiempo = tiempo + stepSize;

        if( fabs(pre_errorsurge) < rango_dist_objetivo) haLlegado = true;

    }


    newControl->components[1]->as<control::RealVectorControlSpace::ControlType>()->values[0] = sumTau0 / tiempo;
    newControl->components[1]->as<control::RealVectorControlSpace::ControlType>()->values[1] = sumTau1 / tiempo;
    newControl->components[1]->as<control::RealVectorControlSpace::ControlType>()->values[2] = sumTau2 / tiempo;
    newControl->components[1]->as<control::RealVectorControlSpace::ControlType>()->values[3] = tiempo;

    
    si_->copyState(dest, stateTemp); //Si se descomenta la parte de abajo, quitar esta linea
    si_->freeState(stateTemp);
    si_->freeState(stateRes);
    cSpace_->freeControl(tempControl);

    steps = steps + (int) tiempo/stepSize;

    // Propagate the first control, and find how far it is from the target state
    //base::State *bestState   = si_->allocState();    
    //steps = si_->propagateWhileValid(source, newControl, steps, bestState);

    //si_->copyState(dest, bestState);
    //si_->freeState(bestState);

    return steps;
}


double ompl::auvplanning::AUVDirectedControlSampler::pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw){

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

    return output;

}