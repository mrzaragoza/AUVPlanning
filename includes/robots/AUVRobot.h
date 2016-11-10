#ifndef AUVROBOT_H_
#define AUVROBOT_H_

#include <array>
#include <string.h>
#include <math.h>
#include <limits>

#include "robots/dynamics/AUVdynamics.h"
#include "colisionador/RigidBodyGeometry.h"
#include "colisionador/GeometrySpecification.h"
#include <colisionador/appUtil.h>
#include "robots/dynamics/ODEAUVSolver.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/SimpleDirectedControlSampler.h"
#include "planners/PlanificadorLocal/AUVSemiRandomDirectedControlSampler.h"
#include "planners/PlanificadorLocal/AUVDirectedControlSampler.h"
#include "planners/PlanificadorLocal/AUVPIDControlSampler.h"

using namespace std;
using namespace ompl;

#define AUV_SIMPLE_DCS        0
#define AUV_SEMI_RANDOM_DCS   1
#define AUV_2PID_DCS          2
#define AUV_PID_DCS           3
#define AUV_DIRECTED_DCS      4


namespace ompl
{
  namespace auvplanning
  {
    OMPL_CLASS_FORWARD(AUVRobot);

    class AUVRobot{
    public:
     AUVRobot(bool compound, YAML::Node config);
     ~AUVRobot(){}

     void setup(int type);

     base::ScopedState<> getDefaultStartState(void) const;
     void setDefaultControlBounds();
     void setDefaultCompoundControlBounds();

     base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const;

     base::ScopedState<> getGeometricComponentState(const base::ScopedState<> &state, unsigned int index) const
     {
      return base::ScopedState<>(getGeometricComponentStateSpace(), getGeometricComponentStateInternal(state.get(), index));
    }


    const base::StateSpacePtr& getGeometricComponentStateSpace(void) const{ 
      return geometricStateSpace;                   
    }

    auvplanning::GeometricStateExtractor getGeometricStateExtractor(void) const
    {
      return boost::bind(&AUVRobot::getGeometricComponentStateInternal, this, _1, _2);
    }

   //getters & setters
    control::SimpleSetup&  getSimpleSetup(){                   return ss_;           }            
    auvplanning::RigidBodyGeometry&  getRigidBodyGeometry(){   return rbg_;          }            
    base::StateValidityCheckerPtr&  getStateValidityChecker(){ return validitySvc_;  }
    void setDirectedControlSampler(int type);


  protected:

    //static ompl::control::SpaceInformationPtr AUVSpaceInformationAllocator(bool compound);
    //static ompl::control::DirectedControlSamplerPtr AUV2StepPIDControlSamplerAllocator(const ompl::control::SpaceInformation *si, unsigned int k, YAML::Node config);
    static ompl::control::DirectedControlSamplerPtr AUVPIDControlSamplerAllocator(const ompl::control::SpaceInformation *si, unsigned int k, YAML::Node config);
    static ompl::control::DirectedControlSamplerPtr AUVDirectedControlSamplerAllocator(const ompl::control::SpaceInformation *si, unsigned int k, YAML::Node config);
    static ompl::control::DirectedControlSamplerPtr AUVSemiRandomDirectedControlSamplerAllocator(const ompl::control::SpaceInformation *si, unsigned int k, YAML::Node config);
    static ompl::control::DirectedControlSamplerPtr RandomDirectedControlSamplerAllocator(const ompl::control::SpaceInformation *si, unsigned int k);

    const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
    {
      base::State *state_;
      state_ = geometricStateSpace->allocState();
      state_->as<base::RealVectorStateSpace::StateType>()->values[0] = state->as<base::RealVectorStateSpace::StateType>()->values[0];
      state_->as<base::RealVectorStateSpace::StateType>()->values[1] = state->as<base::RealVectorStateSpace::StateType>()->values[1];
      state_->as<base::RealVectorStateSpace::StateType>()->values[2] = state->as<base::RealVectorStateSpace::StateType>()->values[2];
      return state_;
    }

    static control::ControlSpacePtr constructControlSpace(bool compound)
    {
      if(compound){
        control::ControlSpacePtr controlspace(new control::CompoundControlSpace(constructStateSpace()));
        controlspace->as<control::CompoundControlSpace>()->addSubspace(control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 4)));
        controlspace->as<control::CompoundControlSpace>()->addSubspace(control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 4)));
        return controlspace;
      }

      return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 3));
    }

    static base::StateSpacePtr constructStateSpace(void)
    {
      return base::StateSpacePtr(new base::RealVectorStateSpace(8));
    }

  private:

    std::string                             name_;

    const control::SpaceInformationPtr      sinf_;

    control::SimpleSetup                    ss_;

    auvplanning::RigidBodyGeometry          rbg_;

    base::StateValidityCheckerPtr           validitySvc_;

    auvplanning::AUVDynamicsPtr             dynamics_;

    control::ODESolverPtr                   odeSolver_;

    base::StateSpacePtr                     geometricStateSpace;

    control::DirectedControlSamplerPtr      dControlSampler_;

    YAML::Node                              config_;
  };

}
}
#endif /* AUVROBOT_H_ */
