
#include "ompl/control/Control.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>

#include <boost/version.hpp>
#if BOOST_VERSION >= 105300
#include <boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::odeint;
#else
#include <omplext_odeint/boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::omplext_odeint;
#endif
#include <boost/function.hpp>
#include <cassert>
#include <vector>


using namespace ompl;


void odeQuadrotor(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{
    const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    double massInv_;
    double beta_;


    // zero out qdot
    qdot.resize (q.size (), 0);

    // derivative of position
    qdot[0] = q[7];
    qdot[1] = q[8];
    qdot[2] = q[9];

    // derivative of orientation
    // 1. First convert omega to quaternion: qdot = omega * q / 2
    base::SO3StateSpace::StateType qomega;
    qomega.w = 0;
    qomega.x = .5*q[10];
    qomega.y = .5*q[11];
    qomega.z = .5*q[12];

    // 2. We include a numerical correction so that dot(q,qdot) = 0. This constraint is
    // obtained by differentiating q * q_conj = 1
    double delta = q[3] * qomega.x + q[4] * qomega.y + q[5] * qomega.z;

    // 3. Finally, set the derivative of orientation
    qdot[3] = qomega.x - delta * q[3];
    qdot[4] = qomega.y - delta * q[4];
    qdot[5] = qomega.z - delta * q[5];
    qdot[6] = qomega.w - delta * q[6];

    // derivative of velocity
    // the z-axis of the body frame in world coordinates is equal to
    // (2(wy+xz), 2(yz-wx), w^2-x^2-y^2+z^2).
    // This can be easily verified by working out q * (0,0,1).
    qdot[7] = massInv_ * (-2*u[0]*(q[6]*q[4] + q[3]*q[5]) - beta_ * q[7]);
    qdot[8] = massInv_ * (-2*u[0]*(q[4]*q[5] - q[6]*q[3]) - beta_ * q[8]);
    qdot[9] = massInv_ * (  -u[0]*(q[6]*q[6]-q[3]*q[3]-q[4]*q[4]+q[5]*q[5]) - beta_ * q[9]) - 9.81;

    // derivative of rotational velocity
    qdot[10] = u[1];
    qdot[11] = u[2];
    qdot[12] = u[3];
}





void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
{
    const double *tau = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize (q.size (), 0);

    // q = [x, y, z, roll, pitch, yaw, xdot, ydot, zdot, rolldot, pitchdot, yawdot] = [x, y, z, roll, pitch, yaw, u, v, w, p, q, r]

    int m = 160;
    int l1 = 1;
    int l2 = 1;
    
    int I_zz = 15;
    
    int X_u = 60;
    int Z_w = 100;
    int N_r = 20;

    double velocities[] = {q[6], q[7], q[8], q[9], q[10], q[11]};

    int Xu_u = 90;
    int Zw_w = 150;
    int Nr_r = 15;

    double X_udot = 0.1 * m;
    double Z_wdot = 0.1 * m;
    double N_rdot = 0.1 * I_zz;

    double g = 9.81;
    int fluid_density = 998;
    int vol_fluid_displaced = 151;

    double W = m * g;
    double B = fluid_density * g * vol_fluid_displaced;

    // derivatives in position and orientation
    qdot[0] = velocities[0]; //xdot    
    qdot[1] = velocities[1]; //ydot
    qdot[2] = velocities[2]; //zdot
    qdot[3] = velocities[3]; //rolldot
    qdot[4] = velocities[4]; //pitchdot
    qdot[5] = velocities[5]; //yawdot

    // derivatives of the velocity in the AUV frame

    double ac_surge = ( (tau[1] + tau[2]) - (X_u + Xu_u * abs(velocities[0])) * velocities[0] ) / (m + X_udot);
    double ac_heave = ( -tau[0] - (Z_w + Zw_w * abs(velocities[2])) * velocities[2] - (W - B) * q[2] ) / (m + Z_wdot);
    double ac_yaw = ( (l1 * tau[1] - l2 * tau[2]) - (N_r + Nr_r * abs(velocities[5])) * velocities[5]) / (I_zz + N_rdot);

    // derivatives of the velocity in the World frame

    qdot[6] = ac_surge * cos(q[5]);
    qdot[7] = ac_surge * sin(q[5]);
    qdot[8] = ac_heave;
    qdot[9] = 0;    
    qdot[10] = 0;
    qdot[11] = ac_yaw;

}

void postPropagate(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State* result)
{
    // Normalize orientation value between 0 and 2*pi
    /*const ompl::base::SO2StateSpace* SO2 = getStateSpace()->as<ompl::base::SE2StateSpace>()->as<ompl::base::SO2StateSpace>(1);
    ompl::base::SO2StateSpace::StateType* so2 = result->as<ompl::base::SE2StateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);*/
}

/// @cond IGNORE
class DemoControlSpace : public control::RealVectorControlSpace
{
public:

    DemoControlSpace(const base::StateSpacePtr &stateSpace) : control::RealVectorControlSpace(stateSpace, 3)
    {
    }
};
/// @endcond


int main(int argc, char**){

    /// construct the state space we are planning in
    base::StateSpacePtr space(new base::SE3StateSpace());

    /// set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(3);
    bounds.setLow(-20);
    bounds.setHigh(20);

    space->as<base::SE3StateSpace>()->setBounds(bounds);

    // create a control space
    control::ControlSpacePtr cspace(new DemoControlSpace(space));

    // set the bounds for the control space
    base::RealVectorBounds cbounds(3);
    cbounds.setLow(-3);
    cbounds.setHigh(3);

    cspace->as<DemoControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    control::SimpleSetup setup(cspace);

    control::ODESolverPtr odeSolver(new control::ODEBasicSolver<>(setup.getSpaceInformation(), &ode));

    //control::ODESolver::StateType state;// = new control::ODESolver::StateType();
    //for (int i = 0; i < 12; ++i) state.push_back(0);
    //const double *tau = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

    base::ScopedState<base::SE3StateSpace> start(space);
    start->setX(0.);
    start->setY(0.);
    start->setZ(0.);
    start->rotation().setIdentity();

    control::Control *control = new control::CompoundControl();

    //control::ODESolver::StateType stateResult;
    base::ScopedState<base::SE3StateSpace> stateResult(space);

    setup.setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, &postPropagate));

    //control::StatePropagatorPtr stPropagator = setup.getStatePropagator();

    //stPropagator->propagate(state, control, 5.0);
    //void propagate(const base::State *state, const Control *control, int steps, base::State *result) const;
    setup.getSpaceInformation()->propagate(start,control,10,stateResult);

    return 0;
}