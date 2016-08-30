#ifndef AUV_TORPEDO4D_H_
#define AUV_TORPEDO4D_H_

#include <array>
#include <math.h>
#include "colisionador/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

using namespace std;

#define GRAVITY 9.81
#define FLUID_DENSITY 1025
#define N_DIMENSIONS 4

class AUV_Torpedo : public ompl::guillermo::AppBase<ompl::guillermo::CONTROL>{
    public:
       AUV_Torpedo();
       AUV_Torpedo(const ompl::control::ControlSpacePtr &controlSpace);
       ~AUV_Torpedo(){}

       bool isSelfCollisionEnabled(void) const{                                                     return false;                }
       virtual unsigned int getRobotCount(void) const{                                              return 1;                    }

       virtual ompl::base::ScopedState<> getDefaultStartState(void) const;
       virtual ompl::base::ScopedState<> getFullStateFromGeometricComponent(const ompl::base::ScopedState<> &state) const;
       virtual const ompl::base::StateSpacePtr& getGeometricComponentStateSpace(void) const{ 
            return geometricStateSpace;
            //return getStateSpace();//->as<ompl::base::RealVectorStateSpace>();                    
       }

       array<double,4> getVehicleLengths(){                                                         return lengths_;                           }
       int             getMass(){                                                                   return mass_;                              }
       array<double,N_DIMENSIONS> getDampingCoefficients(){                                            return dampingCoefficients_;               }
       array<double,N_DIMENSIONS> getQuadraticDampingCoefficients(){                                   return quadraticDampingCoefficients_;      }
       array<double,N_DIMENSIONS> getRBMassCoefficients(){                                             return rbMassCoefficients_;                }
       array<double,N_DIMENSIONS> getAddedMassCoefficients(){                                          return addedMassCoefficients_;             }

       void setVehicleLengths(array<double,4> lengths){                                        lengths_ = lengths;                                                     }
       void setMass(int mass){                                                                 mass_ = mass;                                                           }
       void setDampingCoefficients(array<double,N_DIMENSIONS> dampingCoefficients){                       dampingCoefficients_ = dampingCoefficients;                     }
       void setQuadraticDampingCoefficients(array<double,N_DIMENSIONS> quadraticDampingCoefficients){     quadraticDampingCoefficients_ = quadraticDampingCoefficients;   }
       void setRBMassCoefficients(array<double,N_DIMENSIONS> rbMassCoefficients){                         rbMassCoefficients_ = rbMassCoefficients;                       }
       void setAddedMassCoefficients(array<double,N_DIMENSIONS> addedMassCoefficients){                   addedMassCoefficients_ = addedMassCoefficients;                 }

       virtual void setDefaultControlBounds();

       //virtual void setDefaultSpaceBounds();

       virtual void setup(void);

    protected:

        virtual const ompl::base::State* getGeometricComponentStateInternal(const ompl::base::State* state, unsigned int /*index*/) const
        {
            ompl::base::State *state_;
            state_ = geometricStateSpace->allocState();
            state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
            state_->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
            return state_;
            //return state;
        }

        virtual void ode(const ompl::control::ODESolver::StateType& q, const ompl::control::Control *ctrl, ompl::control::ODESolver::StateType& qdot);

        virtual void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result);

        static ompl::control::ControlSpacePtr constructControlSpace(void)
        {
            return ompl::control::ControlSpacePtr(new ompl::control::RealVectorControlSpace(constructStateSpace(), 3));
        }
        static ompl::base::StateSpacePtr constructStateSpace(void)
        {
            return ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(8));
        }

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
        ompl::control::ODESolverPtr odeSolver;


        ompl::base::StateSpacePtr geometricStateSpace;
};


#endif /* AUV_TORPEDO4D_H_ */
