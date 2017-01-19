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

#include "benchmarks/RealVectorStateSpacePropagateDistance.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorStateProjections.h"
#include "ompl/util/Exception.h"
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <cstring>
#include <limits>
#include <cmath>

double ompl::base::RealVectorStateSpacePropagateDistance::distance(const State *state1, const State *state2) const
{
    double dist = 0.0;
    const double *s1 = static_cast<const StateType*>(state1)->values;
    const double *s2 = static_cast<const StateType*>(state2)->values;
    double propState[3] = {static_cast<StateType*>(state1)->values[0],
                            static_cast<StateType*>(state1)->values[1],
                            static_cast<StateType*>(state1)->values[2]};
    (*s1) = (*s1+4);

    velocities[0] =  cos(q[3]) * vel_N + sin(q[3]) * vel_E;
    velocities[1] = -sin(q[3]) * vel_N + cos(q[3]) * vel_E;

    propState[0] = propState[0] + (*s1++) *;
    propState[1] = propState[1] + (*s1++) *;
    propState[2] = propState[2] + (*s1++) *;
    
    double acc_N = ac_surge * cos(q[3]) - ac_sway * sin(q[3]);
    double acc_E = ac_surge * sin(q[3]) + ac_sway * cos(q[3]);

    for (unsigned int i = 0 ; i < 3 ; ++i)
    {
        double diff = propState[i] - (*s2++);
        dist += diff * diff;
    }
    return sqrt(dist);
}

void ompl::base::RealVectorStateSpacePropagateDistance::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const StateType *rfrom = static_cast<const StateType*>(from);
    const StateType *rto = static_cast<const StateType*>(to);
    const StateType *rstate = static_cast<StateType*>(state);
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
        rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
}