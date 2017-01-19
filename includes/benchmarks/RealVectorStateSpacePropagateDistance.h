/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_SPACES_REAL_VECTOR_STATE_SPACE_
#define OMPL_BASE_SPACES_REAL_VECTOR_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <vector>
#include <string>
#include <map>
#include "yaml-cpp/yaml.h"

namespace ompl
{
    namespace base
    {

        /** \brief A state space representing R<sup>n</sup>. The distance function is the L2 norm. */
        class RealVectorStateSpacePropagateDistance : public RealVectorStateSpace
        {
        public:


            /** \brief Constructor. The dimension of of the space needs to be specified. A space representing
                R<sup>dim</sup> will be instantiated */
            RealVectorStateSpacePropagateDistance(unsigned int dim = 0) : RealVectorStateSpace(dim)
            {
                YAML::Node robot_config = YAML::LoadFile("../includes/robots/torpedo.yaml");
                double c_rbm1 = robot_config["torpedo/rbMassCoefficients"][0].as<double>();
                double c_rbm2 = robot_config["torpedo/rbMassCoefficients"][1].as<double>();
                double c_rbm3 = robot_config["torpedo/rbMassCoefficients"][2].as<double>();
                double c_am1 = robot_config["torpedo/aMassCoefficients"][0].as<double>();
                double c_am2 = robot_config["torpedo/aMassCoefficients"][1].as<double>();
                double c_am3 = robot_config["torpedo/aMassCoefficients"][2].as<double>();
                double c_ld1 = robot_config["torpedo/dampingCoefficients"][0].as<double>();
                double c_ld2 = robot_config["torpedo/dampingCoefficients"][1].as<double>();
                double c_ld3 = robot_config["torpedo/dampingCoefficients"][2].as<double>();
                const_desacc_x = (c_rbm1 + c_am1)/(2*c_ld1);
                const_desacc_y = (c_rbm2 + c_am2)/(2*c_ld2);
                const_desacc_z = (c_rbm3 + c_am3)/(2*c_ld3);
            }

            virtual ~RealVectorStateSpacePropagateDistance()
            {
            }

            virtual double distance(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

        private:

            double const_desacc_x;
            double const_desacc_y;
            double const_desacc_z;
        };
    }
}

#endif
