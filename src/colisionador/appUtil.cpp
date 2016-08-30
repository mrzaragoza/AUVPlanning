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
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <limits>

void ompl::guillermo::InferProblemDefinitionBounds(const ompl::base::ProblemDefinitionPtr &pdef, const GeometricStateExtractor &se, double factor, double add,
                                             unsigned int robotCount, const base::StateSpacePtr &space, MotionModel mtype)
{
    // update the bounds based on start states, if needed
    ompl::base::RealVectorBounds bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
    switch(mtype){
        case Motion_2D:
            bounds = space->as<ompl::base::SE2StateSpace>()->getBounds();
            break;
        case Motion_2_5D:
            bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
            break;
        default:
            bounds = space->as<ompl::base::SE3StateSpace>()->getBounds();
            break;
    }

    printf("InferProblemDefinitionBounds-----------------------\n");
    printf("bound x: %f -- %f\n", bounds.low[0], bounds.high[0]);
    printf("bound y: %f -- %f\n", bounds.low[1], bounds.high[1]);
    printf("bound z: %f -- %f\n", bounds.low[2], bounds.high[2]);
    printf("-----------------------\n");

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
            switch(mtype){
                case Motion_2D:
                    x = s->as<ompl::base::SE2StateSpace::StateType>()->getX();
                    y = s->as<ompl::base::SE2StateSpace::StateType>()->getY();
                    z = 0.0;
                    break;
                case Motion_2_5D:
                    x = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                    y = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                    z = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
                    break;
                default:
                    x = s->as<ompl::base::SE3StateSpace::StateType>()->getX();
                    y = s->as<ompl::base::SE3StateSpace::StateType>()->getY();
                    z = s->as<ompl::base::SE3StateSpace::StateType>()->getZ();
                    break;
            }

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

    if (mtype == Motion_3D)
    {
        if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;
        if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;

        space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
    }else if (mtype == Motion_2_5D){
        if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;
        if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;

        space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    }else
        space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    printf("bound x: %f -- %f\n", bounds.low[0], bounds.high[0]);
    printf("bound y: %f -- %f\n", bounds.low[1], bounds.high[1]);
    printf("bound z: %f -- %f\n", bounds.low[2], bounds.high[2]);
    printf("-----------------------\n");
}

void ompl::guillermo::InferEnvironmentBounds(const ompl::base::StateSpacePtr &space, const RigidBodyGeometry &rbg)
{
    MotionModel mtype = rbg.getMotionModel();

    ompl::base::RealVectorBounds bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
    switch(mtype){
        case Motion_2D:
            bounds = space->as<ompl::base::SE2StateSpace>()->getBounds();
            break;
        case Motion_2_5D:
            bounds = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
            break;
        default:
            bounds = space->as<ompl::base::SE3StateSpace>()->getBounds();
            break;
    }
    printf("InferEnvironmentBounds-----------------------\n");
    printf("bound x: %f -- %f\n", bounds.low[0], bounds.high[0]);
    printf("bound y: %f -- %f\n", bounds.low[1], bounds.high[1]);
    printf("bound z: %f -- %f\n", bounds.low[2], bounds.high[2]);
    // if bounds are not valid
    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
    {
        if (mtype == Motion_2D)
            space->as<ompl::base::SE2StateSpace>()->setBounds(rbg.inferEnvironmentBounds());
        else if(mtype == Motion_2_5D)
            space->as<ompl::base::RealVectorStateSpace>()->setBounds(rbg.inferEnvironmentBounds());            
        else
            space->as<ompl::base::SE3StateSpace>()->setBounds(rbg.inferEnvironmentBounds());
    }

    printf("-----------------------\n");
    printf("bound x: %f -- %f\n", space->as<ompl::base::RealVectorStateSpace>()->getBounds().low[0], space->as<ompl::base::RealVectorStateSpace>()->getBounds().high[0]);
    printf("bound y: %f -- %f\n", space->as<ompl::base::RealVectorStateSpace>()->getBounds().low[1], space->as<ompl::base::RealVectorStateSpace>()->getBounds().high[1]);
    printf("bound z: %f -- %f\n", space->as<ompl::base::RealVectorStateSpace>()->getBounds().low[2], space->as<ompl::base::RealVectorStateSpace>()->getBounds().high[2]);
    printf("-----------------------\n");
}

namespace ompl
{
    namespace guillermo
    {
        namespace detail
        {
            class GeometricStateProjector2D : public base::ProjectionEvaluator
            {
            public:

                GeometricStateProjector2D(const base::StateSpacePtr &space, const base::StateSpacePtr &gspace, const GeometricStateExtractor &se) : base::ProjectionEvaluator(space), gm_(gspace->as<base::SE2StateSpace>()), se_(se)
                {
                }

                virtual unsigned int getDimension(void) const
                {
                    return 2;
                }

                virtual void project(const base::State *state, base::EuclideanProjection &projection) const
                {
                    const base::State *gs = se_(state, 0);
                    projection(0) = gs->as<base::SE2StateSpace::StateType>()->getX();
                    projection(1) = gs->as<base::SE2StateSpace::StateType>()->getY();
                }

                virtual void defaultCellSizes(void)
                {
                    bounds_ = gm_->getBounds();
                    const std::vector<double> b = bounds_.getDifference();
                    cellSizes_.resize(2);
                    cellSizes_[0] = b[0] / 20.0;
                    cellSizes_[1] = b[1] / 20.0;
                }

            protected:

                const base::SE2StateSpace *gm_;
                GeometricStateExtractor    se_;

            };

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

            class GeometricStateProjector3D : public base::ProjectionEvaluator
            {
            public:

                GeometricStateProjector3D(const base::StateSpacePtr &space, const base::StateSpacePtr &gspace, const GeometricStateExtractor &se) : base::ProjectionEvaluator(space), gm_(gspace->as<base::SE3StateSpace>()), se_(se)
                {
                }

                virtual unsigned int getDimension(void) const
                {
                    return 3;
                }

                virtual void project(const base::State *state, base::EuclideanProjection &projection) const
                {
                    const base::State *gs = se_(state, 0);
                    projection(0) = gs->as<base::SE3StateSpace::StateType>()->getX();
                    projection(1) = gs->as<base::SE3StateSpace::StateType>()->getY();
                    projection(2) = gs->as<base::SE3StateSpace::StateType>()->getZ();
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

                const base::SE3StateSpace *gm_;
                GeometricStateExtractor    se_;
            };


            // a decomposition is only needed for SyclopRRT and SyclopEST
            class Decomposition2D : public ompl::control::GridDecomposition
            {
            public:
                // 32 x 32 grid
                Decomposition2D(const ompl::base::RealVectorBounds &bounds, const base::StateSpacePtr &space)
                    : GridDecomposition(32, 2, bounds), space_(space), position_(space->getValueLocations()[0])
                {
                }
                virtual void project(const ompl::base::State *s, std::vector<double> &coord) const
                {
                    const double* pos = space_->getValueAddressAtLocation(s, position_);
                    coord.resize(2);
                    coord[0] = pos[0];
                    coord[1] = pos[1];
                }

                virtual void sampleFullState(const ompl::base::StateSamplerPtr &sampler,
                const std::vector<double>& coord, ompl::base::State *s) const
                {
                    double* pos = space_->getValueAddressAtLocation(s, position_);
                    sampler->sampleUniform(s);
                    pos[0] = coord[0];
                    pos[1] = coord[1];
                }

            protected:
                const base::StateSpacePtr space_;
                const ompl::base::StateSpace::ValueLocation& position_;
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

            class Decomposition3D : public ompl::control::GridDecomposition
            {
            public:
                // 16 x 16 x 16 grid
                Decomposition3D(const ompl::base::RealVectorBounds &bounds, const base::StateSpacePtr &space)
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

ompl::base::ProjectionEvaluatorPtr ompl::guillermo::allocGeometricStateProjector(const base::StateSpacePtr &space, MotionModel mtype,
                                                                           const base::StateSpacePtr &gspace, const GeometricStateExtractor &se)
{
    if (mtype == Motion_2D)
        return base::ProjectionEvaluatorPtr(new detail::GeometricStateProjector2D(space, gspace, se));
    else if (mtype == Motion_2_5D)        
        return base::ProjectionEvaluatorPtr(new detail::GeometricStateProjector2_5D(space, gspace, se));
    return base::ProjectionEvaluatorPtr(new detail::GeometricStateProjector3D(space, gspace, se));
}

ompl::control::DecompositionPtr ompl::guillermo::allocDecomposition(const base::StateSpacePtr &space, MotionModel mtype,
    const base::StateSpacePtr &gspace)
{
    // \todo shouldn't this be done automatically?
    const_cast<ompl::base::StateSpace*>(space.get())->computeLocations();

    if (mtype == Motion_2D)
        return control::DecompositionPtr(new detail::Decomposition2D(gspace->as<ompl::base::SE2StateSpace>()->getBounds(), space));
    else if (mtype == Motion_2_5D)        
        return control::DecompositionPtr(new detail::Decomposition2_5D(gspace->as<ompl::base::RealVectorStateSpace>()->getBounds(), space));
    return control::DecompositionPtr(new detail::Decomposition3D(gspace->as<ompl::base::SE3StateSpace>()->getBounds(), space));
}

ompl::base::OptimizationObjectivePtr ompl::guillermo::getOptimizationObjective(
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
