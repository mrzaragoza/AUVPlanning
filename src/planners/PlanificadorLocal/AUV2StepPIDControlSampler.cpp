#include "planners/PlanificadorLocal/AUV2StepPIDControlSampler.h"
#include <ompl/control/ODESolver.h>
#include "robots/dynamics/AUVdynamics.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace oauv = ompl::auvplanning;

ompl::auvplanning::AUV2StepPIDControlSampler::AUV2StepPIDControlSampler(const control::SpaceInformation *si, YAML::Node config) :
control::DirectedControlSampler(si), reference(si->allocState())
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
    
    controlZEstable = robot_config["torpedo/controlZEstable"].as<double>();
    l_motores = robot_config["torpedo/lengths"][2].as<double>();
    max_fuerza_motores = robot_config["torpedo/max_fuerza_motores"].as<double>();

    stepSize = si->getPropagationStepSize();
    stPropagator = si->getStatePropagator();//oc::ODESolver::getStatePropagator(ode_, boost::bind(&oauv::AUVDynamics::postPropagate, dynamics_, _1, _2, _3, _4));

}

ompl::auvplanning::AUV2StepPIDControlSampler::~AUV2StepPIDControlSampler()
{
}

unsigned int ompl::auvplanning::AUV2StepPIDControlSampler::sampleTo(control::Control *control, const base::State *source, base::State *dest)
{
    return propagation(control, source, dest);
}

unsigned int ompl::auvplanning::AUV2StepPIDControlSampler::sampleTo(control::Control *control, const control::Control *previous, const base::State *source, base::State *dest)
{
    return propagation(control, source, dest);
}

/*unsigned int sampleToStates(control::Control *control, const base::State *source, base::State *dest, std::vector<base::State*> istates)
{
    return propagation(control, source, dest, istates);
}*/

unsigned int ompl::auvplanning::AUV2StepPIDControlSampler::propagation(control::Control *control, const base::State *source, base::State *dest/*, std::vector<base::State*> istates = NULL*/)
{
    isPIDResetNeeded(source,dest);

    control::RealVectorControlSpace::ControlType *newControl = static_cast<control::RealVectorControlSpace::ControlType*>(control);

    base::State         *stateTemp = si_->allocState();
    base::State         *stateRes = si_->allocState();
    si_->copyState(stateTemp,source);   

    double              heading,z,yaw;

    //variables de control
    double              fSurge = 0, fZ = 0, mZ = 0, tau0 = 0, tau1 = 0, tau2 = 0;
    unsigned int        tiempo = 0;
    const unsigned int  minDuration = si_->getMinControlDuration();

    double              dist_x = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - 
                                            stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
    double              dist_y = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - 
                                            stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];
    double              z_ref = dest->as<base::RealVectorStateSpace::StateType>()->values[2];    

    double              dist_actual = sqrt(pow(dist_x,2) + pow(dist_y,2));

    bool                hasArrived = (pre_erroryaw != 0 && fabs(pre_erroryaw) < rango_yaw_objetivo) ? true : false;
    /*if (hasArrived)
    {
        printf("hasArrived inicial: si\n");
    }else{
        printf("hasArrived inicial: no\n");
    }*/
    
    bool                hasCollided = false;

    //bool                interpolate = (istates == NULL) ? false : true;

    while(tiempo < minDuration && !hasArrived){
        //printf("getBestControl. en bucle");
        dist_x = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - 
                                            stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
        dist_y = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - 
                                            stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];
        z = stateTemp->as<base::RealVectorStateSpace::StateType>()->values[2];
        yaw = stateTemp->as<base::RealVectorStateSpace::StateType>()->values[3];
        heading = atan2(dist_y,dist_x); //radianes

        fZ = pid(z_ref, z, stepSize, Kpz, Kdz, Kiz, &pre_errorz, &integralz, false);
        mZ = pid(heading, yaw, stepSize, Kpyaw, Kdyaw, Kiyaw, &pre_erroryaw, &integralyaw, true);

        //Traducción de las fuerzas y mometos a las acciones de control
        tau0 = (fZ/max_fuerza_motores) + controlZEstable;
        tau1 = mZ/(2*l_motores*max_fuerza_motores);
        tau2 = -tau1;

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

        if(!si_->isValid(stateRes)){
            //printf("propagation. NO VALIDO en giro!! Tiempo = %d\n", tiempo);
            hasCollided = true;
            break;
        }
        
        //haEntrado = true;
        si_->copyState(stateTemp,stateRes);
        /*si_->printState(stateTemp);
        //printf("\r");
        usleep(50);*/

        hasArrived = (fabs(pre_erroryaw) < rango_yaw_objetivo) ? true : false;        
        tiempo++;

        //if(interpolate) istates.add(stateTemp);

    }

    hasArrived = (pre_errorsurge != 0 && (fabs(pre_errorsurge) < rango_dist_objetivo && fabs(pre_errorz) < rango_profundidad_objetivo)) ? true : false;
    double dist_previa = 9999999999999;
    
    /*if (hasArrived)
    {
        printf("hasArrived intermedio: si\n");
    }else{
        printf("hasArrived intermedio: no\n");
    }*/

    while(tiempo < minDuration && !hasArrived && !hasCollided){
        z = stateTemp->as<base::RealVectorStateSpace::StateType>()->values[2];
        dist_x = dest->as<base::RealVectorStateSpace::StateType>()->values[0] - stateTemp->as<base::RealVectorStateSpace::StateType>()->values[0];
        dist_y = dest->as<base::RealVectorStateSpace::StateType>()->values[1] - stateTemp->as<base::RealVectorStateSpace::StateType>()->values[1];
        dist_actual = sqrt(pow(dist_x,2) + pow(dist_y,2));

        fZ = pid(z_ref, z, stepSize, Kpz, Kdz, Kiz, &pre_errorz, &integralz, false);
        fSurge = pid(dist_inicial, dist_inicial-dist_actual, stepSize, Kpsurge, Kdsurge, Kisurge, &pre_errorsurge, &integralsurge, false);

        //Traducción de las fuerzas y mometos a las acciones de control
        tau0 = (fZ/max_fuerza_motores) + controlZEstable;
        tau1 = fSurge/(2*max_fuerza_motores);
        tau2 = fSurge/(2*max_fuerza_motores);

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

        if(!si_->isValid(stateRes)){
            //printf("propagation. NO VALIDO en avance!! Tiempo = %d\n", tiempo);
            hasCollided = true;
            break;
        }
        si_->copyState(stateTemp,stateRes);
        /*si_->printState(stateTemp);
        //printf("\r");
        usleep(50);
        //haEntrado = true;
        if(fabs(pre_errorsurge) < rango_dist_objetivo){
            printf("rango_dist_objetivo %f\n", rango_dist_objetivo);
        }*/

        hasArrived = (fabs(pre_errorsurge) < rango_dist_objetivo && fabs(pre_errorz) < rango_profundidad_objetivo) ? true : false;
        
        hasArrived = (hasArrived || dist_previa < dist_actual) ? true : false;
        dist_previa = dist_actual;

        tiempo++;

        //if(interpolate) istates.add(stateTemp);
    }

    //if(!haEntrado) printf("NO HA ENTRADO DENTRO DE LOS WHILE!!!!!!!!!!!*******************************************************\n");
    //tiempo = (tiempo>0) ? tiempo-1 : 0;
    si_->copyState(dest, stateTemp);
    si_->freeState(stateTemp);
    si_->freeState(stateRes);
    return tiempo;
}


double ompl::auvplanning::AUV2StepPIDControlSampler::pid(double reference, double value, double dt, double Kp, double Kd, double Ki, double *pre_error, double *integral, bool isYaw){

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

    if(output > 5) output = 5;
    else if(output < -5) output = -5;

    *pre_error = error;

    return output;

}

void ompl::auvplanning::AUV2StepPIDControlSampler::isPIDResetNeeded(const base::State *init, const base::State *dest){

    /*si_->printState(reference);
    si_->printState(dest);
    printf("isPIDResetNeeded\n");*/
    if(si_->equalStates(reference,dest) == false){
        si_->copyState(reference, dest);

        //Distancias con estado inicial
        double dist_x_inicial       = reference->as<base::RealVectorStateSpace::StateType>()->values[0] - 
                                            init->as<base::RealVectorStateSpace::StateType>()->values[0];
        double dist_y_inicial       = reference->as<base::RealVectorStateSpace::StateType>()->values[1] - 
                                            init->as<base::RealVectorStateSpace::StateType>()->values[1];

        dist_inicial         = sqrt(pow(dist_x_inicial,2) + pow(dist_y_inicial,2));
        
        pre_errorz = 0;
        pre_errorsurge = 0;
        pre_erroryaw = 0;
        integralz = 0;
        integralsurge = 0;
        integralyaw = 0;
        //printf("\t\tReset done: %f %f %f %f %f %f \n",pre_errorz,pre_errorsurge,pre_erroryaw,integralz,integralsurge,integralyaw);
    }/*else{
        printf("\t\tVariables: %f %f %f %f %f %f \n",pre_errorz,pre_errorsurge,pre_erroryaw,integralz,integralsurge,integralyaw);
    }*/
}