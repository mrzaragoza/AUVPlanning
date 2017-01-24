#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/planners/syclop/Decomposition.h>
#include "colisionador/RigidBodyGeometry.h"

namespace ompl
{

    namespace auvplanning
    {

        void InferProblemDefinitionBounds(const base::ProblemDefinitionPtr &pdef, const GeometricStateExtractor &se, double factor, double add,
                                          unsigned int robotCount, const base::StateSpacePtr &space);
        void InferEnvironmentBounds(const base::StateSpacePtr &space, const RigidBodyGeometry &rbg);

        base::ProjectionEvaluatorPtr allocGeometricStateProjector(const base::StateSpacePtr &space,
                                                                  const base::StateSpacePtr &gspace, const GeometricStateExtractor &se);

        control::DecompositionPtr allocDecomposition(const base::StateSpacePtr &space,
            const base::StateSpacePtr &gspace);

        ompl::base::OptimizationObjectivePtr getOptimizationObjective(const base::SpaceInformationPtr &si, const std::string &objective, double threshold);

    }

}
