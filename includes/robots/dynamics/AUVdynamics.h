#ifndef AUVDYNAMICS_H_
#define AUVDYNAMICS_H_

#include <array>
#include <math.h>
#include "yaml-cpp/yaml.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/ODESolver.h>

#define GRAVITY 9.81
#define FLUID_DENSITY 1025
#define N_DIMENSIONS 4

using namespace std;

namespace ompl
{
    namespace auvplanning
    {

        OMPL_CLASS_FORWARD(AUVDynamics);

        class AUVDynamics {
            public:
                AUVDynamics();
                ~AUVDynamics(){}

                void ode(const ompl::control::ODESolver::StateType& q, 
                    const ompl::control::Control *ctrl, ompl::control::ODESolver::StateType& qdot);

                void postPropagate(const ompl::base::State* state, 
                    const ompl::control::Control* control, const double duration, ompl::base::State* result);

            protected:

                double timeStep_;
                array<double,4> lengths_; // L = [long, radius, thruster1, thruster2]
                int mass_;
                array<double,N_DIMENSIONS> dampingCoefficients_; //Dl  = [X_u, Y_v, Z_w, N_r]
                array<double,N_DIMENSIONS> quadraticDampingCoefficients_; //Dq  = [Xu_u, Yv_v, Zw_w, Nr_r]
                array<double,N_DIMENSIONS> rbMassCoefficients_; //Mrb = [m, m, m, Izz]
                array<double,N_DIMENSIONS> addedMassCoefficients_; //Ma  = [X_udot, Y_vdot, Z_wdot, N_rdot]

                double vol_fluid_displaced;
                int long_fluid_displaced;
                double f_espuma;
                double W;
                double B;
        };
    }
}


#endif /* AUVDYNAMICS_H_ */