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

#ifndef OMPLAPP_GEOMETRY_RIGID_BODY_GEOMETRY_
#define OMPLAPP_GEOMETRY_RIGID_BODY_GEOMETRY_

#include "omplapp/config.h"
#include "colisionador/GeometrySpecification.h"
#include "colisionador/FCLStateValidityChecker.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#if OMPL_HAS_ASSIMP3
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#else
#include <assimp/aiScene.h>
#include <assimp/assimp.hpp>
#endif
#include <string>
#include <vector>

namespace ompl
{

    namespace auvplanning
    {
        class RigidBodyGeometry
        {
        public:

            /** \brief Constructor expects a state space that can represent a rigid body */
            explicit
            RigidBodyGeometry() : factor_(1.0), add_(0.0)
            {
            }

            virtual ~RigidBodyGeometry(void)
            {
            }

            bool hasEnvironment(void) const
            {
                return !importerEnv_.empty();
            }

            bool hasRobot(void) const
            {
                return !importerRobot_.empty();
            }

            /** \brief Get the robot's center (average of all the vertices of all its parts) */
            aiVector3D getRobotCenter(unsigned int robotIndex) const;

            /** \brief This function specifies the name of the CAD
                file representing the environment (\e
                env). Returns 1 on success, 0 on failure. */
            virtual bool setEnvironmentMesh(const std::string &env);

             /** \brief This function specifies the name of the CAD
                 file representing the robot (\e robot). Returns 1 on success, 0 on failure. */
            virtual bool setRobotMesh(const std::string &robot);

            /** \brief Allocate default state validity checker using PQP. */
            const base::StateValidityCheckerPtr& allocStateValidityChecker(const base::SpaceInformationPtr &si, const GeometricStateExtractor &se, bool selfCollision);

            const GeometrySpecification& getGeometrySpecification(void) const;

            /** \brief The bounds of the environment are inferred
                based on the axis-aligned bounding box for the objects
                in the environment. The inferred size is multiplied by
                \e factor. By default \e factor = 1, */
            void setBoundsFactor(double factor)
            {
                factor_ = factor;
            }

            /** \brief Get the data set by setBoundsFactor() */
            double getBoundsFactor(void) const
            {
                return factor_;
            }

            /** \brief The bounds of the environment are inferred
                based on the axis-aligned bounding box for the objects
                in the environment. \e add is added to the inferred
                size. By default \e add = 0, */
            void setBoundsAddition(double add)
            {
                add_ = add;
            }

            /** \brief Get the data set by setBoundsAddition() */
            double getBoundsAddition(void) const
            {
                return add_;
            }

            /** \brief Given the representation of an environment,
                infer its bounds. The bounds will be 2-dimensional
                when planning in 2D and 3-dimensional when planning in
                3D. */
            base::RealVectorBounds inferEnvironmentBounds(void) const;

        protected:

            void computeGeometrySpecification(void);

            /** \brief The factor to multiply inferred environment bounds by (default 1) */
            double              factor_;

            /** \brief The value to add to inferred environment bounds (default 0) */
            double              add_;

            /** \brief Instance of assimp importer used to load environment */
            std::vector< boost::shared_ptr<Assimp::Importer> > importerEnv_;

            /** \brief Instance of assimp importer used to load robot */
            std::vector< boost::shared_ptr<Assimp::Importer> > importerRobot_;

            /** \brief Object containing mesh data for robot and environment */
            GeometrySpecification         geom_;

            /** \brief Instance of the state validity checker for collision checking */
            base::StateValidityCheckerPtr validitySvc_;

        };

    }
}

#endif
