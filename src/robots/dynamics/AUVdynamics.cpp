#include "robots/dynamics/AUVdynamics.h"


using namespace ompl;

ompl::auvplanning::AUVDynamics::AUVDynamics()
{
    YAML::Node config = YAML::LoadFile("../includes/robots/torpedo.yaml");
    
    mass_ = config["torpedo/mass"].as<int>(); //34.5
    lengths_ = {config["torpedo/lengths"][0].as<double>(), 
        config["torpedo/lengths"][1].as<double>(), 
        config["torpedo/lengths"][2].as<double>(), 
        config["torpedo/lengths"][3].as<double>()};

    long_fluid_displaced = config["torpedo/long_fluid_displaced"].as<double>();
    //vol_fluid_displaced = M_PI * pow(lengths_[1],2) * long_fluid_displaced;
    vol_fluid_displaced = config["torpedo/vol_fluid_displaced"].as<double>();
    f_espuma = config["torpedo/f_espuma"].as<double>();

    W = mass_ /** GRAVITY*/;
    //B = (double) GRAVITY * (FLUID_DENSITY * vol_fluid_displaced + f_espuma);
    B = (double) /*GRAVITY **/ (vol_fluid_displaced + f_espuma);
    //printf("W:%f  B:%f  W-B:%f\n",W, B, W-B );
    rbMassCoefficients_ = {config["torpedo/rbMassCoefficients"][0].as<double>(),
        config["torpedo/rbMassCoefficients"][1].as<double>(),
        config["torpedo/rbMassCoefficients"][2].as<double>(),
        config["torpedo/rbMassCoefficients"][3].as<double>()};
    
    addedMassCoefficients_ = {config["torpedo/aMassCoefficients"][0].as<double>(),
        config["torpedo/aMassCoefficients"][1].as<double>(),
        config["torpedo/aMassCoefficients"][2].as<double>(),
        config["torpedo/aMassCoefficients"][3].as<double>()};

    dampingCoefficients_ = {config["torpedo/dampingCoefficients"][0].as<double>(),
        config["torpedo/dampingCoefficients"][1].as<double>(),
        config["torpedo/dampingCoefficients"][2].as<double>(),
        config["torpedo/dampingCoefficients"][3].as<double>()};

    quadraticDampingCoefficients_ = {config["torpedo/quadraticDampingCoefficients"][0].as<double>(),
        config["torpedo/quadraticDampingCoefficients"][1].as<double>(),
        config["torpedo/quadraticDampingCoefficients"][2].as<double>(),
        config["torpedo/quadraticDampingCoefficients"][3].as<double>()};

    f_th = config["torpedo/max_fuerza_motores"].as<double>();
}

void ompl::auvplanning::AUVDynamics::ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{

    /*printf("ENTRA EN AUVDynamics::ode \n");
    fflush(stdout);*/
    const double *tau_in = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;
    // zero out qdot
    qdot.resize (q.size (), 0);

    //Se multiplica la acción de control por la fuerza que puede dar cada thruster.
    array<double,3> tau;
    //double f = 5.0;
    tau[0] = tau_in[0] * f_th; 
    tau[1] = tau_in[1] * f_th;
    tau[2] = tau_in[2] * f_th;
    double velocities[] = {q[4], q[5], q[6], q[7]};

    // derivatives in position and orientation
    qdot[0] = velocities[0]; //xdot    
    qdot[1] = velocities[1]; //ydot
    qdot[2] = velocities[2]; //zdot
    qdot[3] = velocities[3]; //yawdot
    ////////////////////
    double vel_N = velocities[0];
    double vel_E = velocities[1];
    velocities[0] =  cos(q[3]) * vel_N + sin(q[3]) * vel_E;
    velocities[1] = -sin(q[3]) * vel_N + cos(q[3]) * vel_E;


    ////////////////////
    // derivatives of the velocity in the AUV frame
    double ac_surge = ( ( tau[1] + tau[2] ) -    (dampingCoefficients_[0] + quadraticDampingCoefficients_[0] * fabs(velocities[0])) * velocities[0] ) 
                        / (rbMassCoefficients_[0] + addedMassCoefficients_[0]);
    double ac_sway  = ( 0 -    (dampingCoefficients_[1] + quadraticDampingCoefficients_[1] * fabs(velocities[1])) * velocities[1] ) 
                        / (rbMassCoefficients_[1] + addedMassCoefficients_[1]);
    double ac_heave = ( tau[0] - (dampingCoefficients_[2] + quadraticDampingCoefficients_[2] * fabs(velocities[2])) * velocities[2] + (W - B) ) 
                        / (rbMassCoefficients_[2] + addedMassCoefficients_[2]);
    double ac_yaw   = ( ( lengths_[2]*tau[1] - lengths_[3]*tau[2] ) - (dampingCoefficients_[3] + quadraticDampingCoefficients_[3] * fabs(velocities[3])) * velocities[3]) 
                        / (rbMassCoefficients_[3] + addedMassCoefficients_[3]);
    // derivatives of the velocity in the World frame

    ////////////////////
    double acc_N = ac_surge * cos(q[3]) - ac_sway * sin(q[3]);
    double acc_E = ac_surge * sin(q[3]) + ac_sway * cos(q[3]);
    ////////////////////
    qdot[4] = acc_N;
    qdot[5] = acc_E;
    qdot[6] = ac_heave;
    qdot[7] = ac_yaw;

    /*printf("AUVDynamics::ode q %f %f %f %f %f %f %f %f \n",q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
    fflush(stdout);
    printf("AUVDynamics::ode qdot %f %f %f %f %f %f %f %f \n",qdot[0],qdot[1],qdot[2],qdot[3],qdot[4],qdot[5],qdot[6],qdot[7]);
    fflush(stdout);
    printf("AUVDynamics::ode tau %f %f %f \n",tau[0],tau[1],tau[2]);
    fflush(stdout);

    printf("SALE DE AUVDynamics::ode \n");
    fflush(stdout);*/
}

void ompl::auvplanning::AUVDynamics::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    // Normalize orientation value between -pi and pi
    base::RealVectorStateSpace::StateType* state_ = result->as<base::RealVectorStateSpace::StateType>();
    state_->values[3] = std::fmod(state_->values[3],2*M_PI);

    if(state_->values[3] > M_PI && state_->values[3] < 2*M_PI)
        state_->values[3] = state_->values[3] - 2*M_PI;
    else if (state_->values[3] < -M_PI && state_->values[3] > - 2*M_PI)
        state_->values[3] = state_->values[3] + 2*M_PI;

}