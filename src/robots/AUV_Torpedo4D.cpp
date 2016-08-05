#include "robots/AUV_Torpedo4D.h"

using namespace ompl;

AUV_Torpedo::AUV_Torpedo()
    : guillermo::AppBase<guillermo::CONTROL>(constructControlSpace(), guillermo::Motion_2_5D), timeStep_(1e-2), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&AUV_Torpedo::ode, this, _1, _2, _3)))
{
    name_ = std::string("Path planning test for a 4 dimensional space");

    //Valores Sparus II
    
    setMass(63.2); //34.5
    setVehicleLengths({1.615, 0.11, -0.30, 0.30});
    vol_fluid_displaced = M_PI * pow(lengths_[1],2) * lengths_[0];
    setRBMassCoefficients({mass_, mass_, mass_, 8});
    array<int,N_DIMENSIONS> amCoef {0, 0, 0, 0};
    for (int i = 0; i < N_DIMENSIONS; ++i)
    {
        amCoef[i] = round(0.1 * rbMassCoefficients_[i]);
    }
    setAddedMassCoefficients(amCoef); //La matriz de masas añadidas la tienen mal, ésta de aquí está sacada del Paper
    setDampingCoefficients({20, 60, 60, 8});
    setQuadraticDampingCoefficients({30, 60, 60, 10});
    
    setDefaultControlBounds();

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&AUV_Torpedo::postPropagate, this, _1, _2, _3, _4)));
}

AUV_Torpedo::AUV_Torpedo(const control::ControlSpacePtr &controlSpace)
    : guillermo::AppBase<guillermo::CONTROL>(controlSpace, guillermo::Motion_2_5D), timeStep_(1e-2), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&AUV_Torpedo::ode, this, _1, _2, _3)))
{
    setDefaultControlBounds();

    si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&AUV_Torpedo::postPropagate, this, _1, _2, _3, _4)));
}

ompl::base::ScopedState<> AUV_Torpedo::getFullStateFromGeometricComponent(const ompl::base::ScopedState<> &state) const
{
    /*ompl::base::ScopedState<> s(getStateSpace());
    std::vector <double> reals = state.reals ();

    s = 0.0;
    for (size_t i = 0; i < reals.size (); ++i)
        s[i] = reals[i];
    return s;*/
    return state;
}


base::ScopedState<> AUV_Torpedo::getDefaultStartState(void) const
{
    base::ScopedState<base::RealVectorStateSpace> s(getGeometricComponentStateSpace());
    //aiVector3D s = getRobotCenter(0);

    s[0] = 0.;
    s[1] = 0.;
    s[2] = 0.;
    s[3] = 0.;
    s[4] = 0.;
    s[5] = 0.;
    s[6] = 0.;
    s[7] = 0.;
    return getFullStateFromGeometricComponent(s);
}

//Las acciones de control en los trhusters van de -1 a 1. Siendo 1 el par de fuerza máximo del thruster en el sentido de avance, y -1 el máximo en el contrario.
void AUV_Torpedo::setDefaultControlBounds(void)
{
    base::RealVectorBounds cbounds(3);
    cbounds.setLow(-1.);
    cbounds.setHigh(1.);
    getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(cbounds);
}

 /* namespace ompl */


void AUV_Torpedo::ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{
    const double *tau_in = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize (q.size (), 0);

    //Se multiplica la acción de control por la fuerza que puede dar cada thruster. TODO:(poner en variable global)
    array<double,3> tau;
    double f = 2.0;
    tau[0] = tau_in[0] * f; 
    tau[1] = tau_in[1] * f;
    tau[2] = tau_in[2] * f;

    //printf("q4: %f, q5: %f, q6: %f, q7: %f\n",q[4], q[5], q[6], q[7] );
    double velocities[] = {q[4], q[5], q[6], q[7]};

    double W = mass_ * GRAVITY;
    double B = (double) FLUID_DENSITY * GRAVITY * vol_fluid_displaced;
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
    double ac_surge = ( ( tau[1] + tau[2] ) -    (dampingCoefficients_[0] + quadraticDampingCoefficients_[0] * abs(velocities[0])) * velocities[0] ) 
                        / (rbMassCoefficients_[0] + addedMassCoefficients_[0]);
    double ac_sway  = ( 0 -    (dampingCoefficients_[1] + quadraticDampingCoefficients_[1] * abs(velocities[1])) * velocities[1] ) 
                        / (rbMassCoefficients_[1] + addedMassCoefficients_[1]);
    double ac_heave = ( tau[0] -                 (dampingCoefficients_[2] + quadraticDampingCoefficients_[2] * abs(velocities[2])) * velocities[2] + (W - B) ) 
                        / (rbMassCoefficients_[2] + addedMassCoefficients_[2]);
    double ac_yaw   = ( ( tau[1] - tau[2] ) -    (dampingCoefficients_[3] + quadraticDampingCoefficients_[3] * abs(velocities[3])) * velocities[3]) 
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

}

void AUV_Torpedo::postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    // Normalize orientation value between -pi and pi
    base::RealVectorStateSpace::StateType* state_ = result->as<base::RealVectorStateSpace::StateType>();
    state_->values[3] = std::fmod(state_->values[3],2*M_PI);

    if(state_->values[3] > M_PI && state_->values[3] < 2*M_PI)
        state_->values[3] = state_->values[3] - 2*M_PI;
    else if (state_->values[3] < -M_PI && state_->values[3] > - 2*M_PI)
        state_->values[3] = state_->values[3] + 2*M_PI;

    //if(state_->values[2] < 0) state_->values[2] = 0.0;

    //stateSpace->enforceBounds(state_);
}