#include "benchmarks/RealVectorStateSpaceSimplePropagateDistance.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Exception.h"
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <cstring>
#include <limits>
#include <cmath>

double ompl::base::RealVectorStateSpaceSimplePropagateDistance::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    double dist = 0.0;
    const double *s1 = static_cast<const StateType*>(state1)->values;
    const double *s2 = static_cast<const StateType*>(state2)->values;
    double propState[3] = {*(s2),*(s2+1),*(s2+2)};

    propState[0] = propState[0] - *(s2+4) * *(s2+4) / const_desacc_x;
    propState[1] = propState[1] - *(s2+5) * *(s2+5) / const_desacc_y;
    propState[2] = propState[2] - *(s2+6) * *(s2+6) / const_desacc_z;

    for (unsigned int i = 0 ; i < 3 ; ++i)
    {
        double diff = propState[i] - (*s1++);
        dist += diff * diff;
    }
    return sqrt(dist);
}

/*void ompl::base::RealVectorStateSpaceSimplePropagateDistance::freeState(State *state) const
{
    StateType *rstate = static_cast<StateType*>(state);
    delete[] rstate->values;
    delete rstate;
}*/
