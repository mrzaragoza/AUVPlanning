#ifndef REAL_VECTOR_STATE_SPACE_SIMPLE_PROPAGATE_DISTANCE_
#define REAL_VECTOR_STATE_SPACE_SIMPLE_PROPAGATE_DISTANCE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <vector>
#include <string>
#include <map>
#include "yaml-cpp/yaml.h"

namespace ompl
{
    namespace base
    {

        class RealVectorStateSpaceSimplePropagateDistance : public RealVectorStateSpace
        {
        public:

            RealVectorStateSpaceSimplePropagateDistance(unsigned int dim = 0) : RealVectorStateSpace(dim)
            {
                YAML::Node robot_config = YAML::LoadFile("../includes/robots/torpedo.yaml");
                const_desacc_x = 2 * robot_config["const_dessacc_simple/surge"].as<double>();
                const_desacc_y = 2 * robot_config["const_dessacc_simple/sway"].as<double>();
                const_desacc_z = 2 * robot_config["const_dessacc_simple/heave"].as<double>();
            }

            ~RealVectorStateSpaceSimplePropagateDistance()
            {
            }

            double distance(const State *state1, const State *state2) const;
            //void freeState(State *state) const;

        private:

            double const_desacc_x;
            double const_desacc_y;
            double const_desacc_z;
        };
    }
}

#endif
