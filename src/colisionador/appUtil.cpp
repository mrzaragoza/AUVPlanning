/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include "colisionador/appUtil.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <limits>

void ompl::auvplanning::InferProblemDefinitionBounds(const ompl::base::ProblemDefinitionPtr &pdef, const GeometricStateExtractor &se, double factor, double add,
                                             unsigned int robotCount, const base::StateSpacePtr &space)
{
    // update the bounds based on start states, if needed
    ompl::base::RealVectorBounds bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
    bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
            

    /*printf("InferProblemDefinitionBounds-----------------------\n");
    printf("bound x: %f -- %f\n", bounds.low[0], bounds.high[0]);
    printf("bound y: %f -- %f\n", bounds.low[1], bounds.high[1]);
    printf("bound z: %f -- %f\n", bounds.low[2], bounds.high[2]);
    printf("-----------------------\n");*/

    std::vector<const ompl::base::State*> states;
    pdef->getInputStates(states);

    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double minZ = minX;
    double maxX = -minX;
    double maxY = maxX;
    double maxZ = maxX;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
        for (unsigned int r = 0 ; r < robotCount ; ++r)
        {
            const ompl::base::State *s = se(states[i], r);

            double x, y, z;
            
            x = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            y = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
            z = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
                    
            if (minX > x) minX = x;
            if (maxX < x) maxX = x;
            if (minY > y) minY = y;
            if (maxY < y) maxY = y;
            if (minZ > z) minZ = z;
            if (maxZ < z) maxZ = z;
        }
    }
    double dx = (maxX - minX) * (factor - 1.0) + add;
    double dy = (maxY - minY) * (factor - 1.0) + add;
    double dz = (maxZ - minZ) * (factor - 1.0) + add;

    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;

    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;
    
    if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;
    if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;

    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    

    /*printf("bound x: %f -- %f\n", bounds.low[0], bounds.high[0]);
    printf("bound y: %f -- %f\n", bounds.low[1], bounds.high[1]);
    printf("bound z: %f -- %f\n", bounds.low[2], bounds.high[2]);
    printf("-----------------------\n");*/
}

void ompl::auvplanning::InferEnvironmentBounds(const ompl::base::StateSpacePtr &space, const RigidBodyGeometry &rbg)
{
    
    ompl::base::RealVectorBounds bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
    
    bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
            
    /*printf("InferEnvironmentBounds-----------------------\n");
    printf("bound x: %f -- %f\n", bounds.low[0], bounds.high[0]);
    printf("bound y: %f -- %f\n", bounds.low[1], bounds.high[1]);
    printf("bound z: %f -- %f\n", bounds.low[2], bounds.high[2]);*/
    // if bounds are not valid
    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
    {
       space->as<ompl::base::RealVectorStateSpace>()->setBounds(rbg.inferEnvironmentBounds());            
    }

    /*printf("-----------------------\n");
    printf("bound x: %f -- %f\n", space->as<ompl::base::RealVectorStateSpace>()->getBounds().low[0], space->as<ompl::base::RealVectorStateSpace>()->getBounds().high[0]);
    printf("bound y: %f -- %f\n", space->as<ompl::base::RealVectorStateSpace>()->getBounds().low[1], space->as<ompl::base::RealVectorStateSpace>()->getBounds().high[1]);
    printf("bound z: %f -- %f\n", space->as<ompl::base::RealVectorStateSpace>()->getBounds().low[2], space->as<ompl::base::RealVectorStateSpace>()->getBounds().high[2]);
    printf("-----------------------\n");*/
}

namespace ompl
{
    namespace auvplanning
    {
        namespace detail
        {
            
            class GeometricStateProjector2_5D : public base::ProjectionEvaluator
            {
            public:

                GeometricStateProjector2_5D(const base::StateSpacePtr &space, const base::StateSpacePtr &gspace, const GeometricStateExtractor &se) : base::ProjectionEvaluator(space), gm_(gspace->as<base::RealVectorStateSpace>()), se_(se)
                {
                }

                virtual unsigned int getDimension(void) const
                {
                    return 3;
                }

                virtual void project(const base::State *state, base::EuclideanProjection &projection) const
                {
                    const base::State *gs = se_(state, 0);
                    projection(0) = gs->as<base::RealVectorStateSpace::StateType>()->values[0];
                    projection(1) = gs->as<base::RealVectorStateSpace::StateType>()->values[1];
                    projection(2) = gs->as<base::RealVectorStateSpace::StateType>()->values[2];
                }

                virtual void defaultCellSizes(void)
                {
                    bounds_ = gm_->getBounds();
                    const std::vector<double> b = bounds_.getDifference();
                    cellSizes_.resize(3);
                    cellSizes_[0] = b[0] / 20.0;
                    cellSizes_[1] = b[1] / 20.0;
                    cellSizes_[2] = b[2] / 20.0;
                }

            protected:

                const base::RealVectorStateSpace *gm_;
                GeometricStateExtractor           se_;

            };            

            class Decomposition2_5D : public ompl::control::GridDecomposition
            {
            public:
                // 16 x 16 x 16 grid
                Decomposition2_5D(const ompl::base::RealVectorBounds &bounds, const base::StateSpacePtr &space)
                    : GridDecomposition(16, 3, bounds), space_(space), position_(space->getValueLocations()[0])
                {
                }
                virtual void project(const ompl::base::State *s, std::vector<double> &coord) const
                {
                    const double* pos = space_->getValueAddressAtLocation(s, position_);
                    coord.resize(3);
                    coord[0] = pos[0];
                    coord[1] = pos[1];
                    coord[2] = pos[2];
                }

                virtual void sampleFullState(const ompl::base::StateSamplerPtr &sampler,
                const std::vector<double>& coord, ompl::base::State *s) const
                {
                    double* pos = space_->getValueAddressAtLocation(s, position_);
                    sampler->sampleUniform(s);
                    pos[0] = coord[0];
                    pos[1] = coord[1];
                    pos[2] = coord[2];
                }

            protected:
                const base::StateSpacePtr space_;
                const ompl::base::StateSpace::ValueLocation& position_;
            };
        }
    }
}

ompl::base::ProjectionEvaluatorPtr ompl::auvplanning::allocGeometricStateProjector(const base::StateSpacePtr &space,
                                                                           const base::StateSpacePtr &gspace, const GeometricStateExtractor &se)
{
    return base::ProjectionEvaluatorPtr(new detail::GeometricStateProjector2_5D(space, gspace, se));
}

ompl::control::DecompositionPtr ompl::auvplanning::allocDecomposition(const base::StateSpacePtr &space,
    const base::StateSpacePtr &gspace)
{
    // \todo shouldn't this be done automatically?
    const_cast<ompl::base::StateSpace*>(space.get())->computeLocations();

    return control::DecompositionPtr(new detail::Decomposition2_5D(gspace->as<ompl::base::RealVectorStateSpace>()->getBounds(), space));
}

ompl::base::OptimizationObjectivePtr ompl::auvplanning::getOptimizationObjective(
    const base::SpaceInformationPtr &si, const std::string &objective, double threshold)
{
    base::OptimizationObjectivePtr obj;
    if (objective == std::string("length"))
        obj.reset(new base::PathLengthOptimizationObjective(si));
    else if (objective == std::string("max min clearance"))
        obj.reset(new base::MaximizeMinClearanceObjective(si));
    else if (objective == std::string("mechanical work"))
        obj.reset(new base::MechanicalWorkOptimizationObjective(si));
    else
    {
        OMPL_WARN("ompl::app::getOptimizationObjective: unknown optimization objective called \"%s\"; using \"length\" instead", objective.c_str());
        obj.reset(new base::PathLengthOptimizationObjective(si));
    }
    obj->setCostThreshold(base::Cost(threshold));
    return obj;
}
