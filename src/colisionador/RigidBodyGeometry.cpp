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

#include "colisionador/RigidBodyGeometry.h"

namespace oa = ompl::app;

bool ompl::auvplanning::RigidBodyGeometry::setRobotMesh(const std::string &robot)
{
    importerRobot_.clear();
    computeGeometrySpecification();

    assert(!robot.empty());
    std::size_t p = importerRobot_.size();
    importerRobot_.resize(p + 1);
    importerRobot_[p].reset(new Assimp::Importer());

    const aiScene* robotScene = importerRobot_[p]->ReadFile(robot.c_str(),
                                                            aiProcess_Triangulate            |
                                                            aiProcess_JoinIdenticalVertices  |
                                                            aiProcess_SortByPType            |
                                                            aiProcess_OptimizeGraph          |
                                                            aiProcess_OptimizeMeshes);
    if (robotScene)
    {
        if (!robotScene->HasMeshes())
        {
            OMPL_ERROR("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
            importerRobot_.resize(p);
        }
    }
    else
    {
        printf("%s\n", importerRobot_[p]->GetErrorString());
        OMPL_ERROR("Unable to load robot scene: %s", robot.c_str());
        importerRobot_.resize(p);
    }

    if (p < importerRobot_.size())
    {
        computeGeometrySpecification();
        return true;
    }
    else
        return false;
}

bool ompl::auvplanning::RigidBodyGeometry::setEnvironmentMesh(const std::string &env)
{
    importerEnv_.clear();
    computeGeometrySpecification();

    assert(!env.empty());
    std::size_t p = importerEnv_.size();
    importerEnv_.resize(p + 1);
    importerEnv_[p].reset(new Assimp::Importer());

    const aiScene* envScene = importerEnv_[p]->ReadFile(env.c_str(),
                                                        aiProcess_Triangulate            |
                                                        aiProcess_JoinIdenticalVertices  |
                                                        aiProcess_SortByPType            |
                                                        aiProcess_OptimizeGraph          |
                                                        aiProcess_OptimizeMeshes);
    if (envScene)
    {
        if (!envScene->HasMeshes())
        {
            OMPL_ERROR("There is no mesh specified in the indicated environment resource: %s", env.c_str());
            importerEnv_.resize(p);
        }
    }
    else
    {
        OMPL_ERROR("Unable to load environment scene: %s", env.c_str());
        importerEnv_[p]->GetErrorString();
        importerEnv_.resize(p);
    }

    if (p < importerEnv_.size())
    {
        computeGeometrySpecification();
        return true;
    }
    else
        return false;
}

ompl::base::RealVectorBounds ompl::auvplanning::RigidBodyGeometry::inferEnvironmentBounds(void) const
{
    base::RealVectorBounds bounds(3);

    for (unsigned int i = 0 ; i < importerEnv_.size() ; ++i)
    {
        std::vector<aiVector3D> vertices;
        oa::scene::extractVertices(importerEnv_[i]->GetScene(), vertices);
        oa::scene::inferBounds(bounds, vertices, factor_, add_);
    }

    return bounds;
}

const ompl::auvplanning::GeometrySpecification& ompl::auvplanning::RigidBodyGeometry::getGeometrySpecification(void) const
{
    return geom_;
}

void ompl::auvplanning::RigidBodyGeometry::computeGeometrySpecification(void)
{
    validitySvc_.reset();
    geom_.obstacles.clear();
    geom_.obstaclesShift.clear();
    geom_.robot.clear();
    geom_.robotShift.clear();

    for (unsigned int i = 0 ; i < importerEnv_.size() ; ++i)
        geom_.obstacles.push_back(importerEnv_[i]->GetScene());

    for (unsigned int i = 0 ; i < importerRobot_.size() ; ++i)
    {
        geom_.robot.push_back(importerRobot_[i]->GetScene());
        aiVector3D c = getRobotCenter(i);
        geom_.robotShift.push_back(c);
    }
}

aiVector3D ompl::auvplanning::RigidBodyGeometry::getRobotCenter(unsigned int robotIndex) const
{
    aiVector3D s(0.0, 0.0, 0.0);
    if (robotIndex >= importerRobot_.size())
        throw Exception("Robot " + boost::lexical_cast<std::string>(robotIndex) + " not found.");

    oa::scene::sceneCenter(importerRobot_[robotIndex]->GetScene(), s);
    return s;
}

const ompl::base::StateValidityCheckerPtr& ompl::auvplanning::RigidBodyGeometry::allocStateValidityChecker(const base::SpaceInformationPtr &si, const GeometricStateExtractor &se, bool selfCollision)
{
    if (validitySvc_)
        return validitySvc_;

    GeometrySpecification geom = getGeometrySpecification();
    validitySvc_.reset (new FCLStateValidityChecker(si, geom, se, selfCollision));
               
    return validitySvc_;
}
