#ifndef AUV_TORPEDO_BENCHMARK4D_H_
#define AUV_TORPEDO_BENCHMARK4D_H_


#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <AUV_Torpedo.h>
#include <omplapp/config.h>
#include <boost/math/constants/constants.hpp>


// FCL Headers
#include <fcl/collision.h>
#include <fcl/collision_node.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/continuous_collision.h>

#include <omplapp/geometry/RigidBodyGeometry.h>
#include <omplapp/geometry/detail/assimpUtil.h>
#include <omplapp/geometry/GeometrySpecification.h>

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>





#endif /* AUV_TORPEDO_BENCHMARK4D_H_ */








/*
typedef boost::function<void(fcl::Vec3f&, fcl::Quaternion3f&, const base::State*)> FCLPoseFromStateCallback;

/// \brief The type of geometric bounding done for the robot and environment
typedef fcl::OBBRSS  BVType;
/// \brief The type geometric model used for the meshes
typedef fcl::BVHModel <BVType> Model;

/// \brief Geometric model used for the environment
Model environment_;

/// \brief List of components for the geometric model of the robot
mutable std::vector <Model*> robotParts_;

/// \brief Callback to get the geometric portion of a specific state
oa::GeometricStateExtractor     extractState_;

/// \brief Flag indicating whether the geometry is checked for self collisions
bool                        selfCollision_;

/// \brief Callback to extract translation and rotation from a state
FCLPoseFromStateCallback    poseFromStateCallback_;


/// \brief Checks whether the given robot state collides with the
/// environment or itself.
virtual bool isValid (const base::State *state) const
{
	fcl::CollisionRequest collisionRequest;
	fcl::CollisionResult collisionResult;
	fcl::Quaternion3f rot;
	fcl::Vec3f pos;
	fcl::Transform3f transform;

	if (environment_.num_tris > 0)
	{
		// Performing collision checking with environment.
		for (std::size_t i = 0; i < robotParts_.size(); ++i)
		{
			poseFromStateCallback_(pos, rot, extractState_(state, i));
			transform.setTransform(rot, pos);
			if (fcl::collide(robotParts_[i], transform, &environment_,
					fcl::Transform3f(), collisionRequest, collisionResult) > 0)
				return false;
		}
	}



	return true;
}

/// \brief Check the continuous motion between s1 and s2.  If there is a collision
/// collisionTime will contain the parameterized time to collision in the range [0,1).
virtual bool isValid (const base::State *s1, const base::State *s2, double &collisionTime) const
{
	fcl::Transform3f transi_beg, transi_end, trans;
	fcl::Quaternion3f rot;
	fcl::Vec3f pos;
	fcl::ContinuousCollisionRequest collisionRequest(10, 0.0001, fcl::CCDM_SCREW,
			fcl::GST_LIBCCD, fcl::CCDC_CONSERVATIVE_ADVANCEMENT);
	fcl::ContinuousCollisionResult collisionResult;

	// Checking for collision with environment
	if (environment_.num_tris > 0)
	{
		for (size_t i = 0; i < robotParts_.size(); ++i)
		{
			// Getting the translation and rotation from s1 and s2
			poseFromStateCallback_(pos, rot, extractState_(s1, i));
			transi_beg.setTransform(rot, pos);
			poseFromStateCallback_(pos, rot, extractState_(s2, i));
			transi_end.setTransform(rot, pos);

			// Checking for collision
			fcl::continuousCollide(robotParts_[i], transi_beg, transi_end,
					&environment_, trans, trans,
					collisionRequest, collisionResult);
			if (collisionResult.is_collide)
			{
				collisionTime = collisionResult.time_of_contact;
				return false;
			}
		}
	}

	// Checking for self collision
	if (selfCollision_)
	{
		fcl::Transform3f transj_beg, transj_end;
		for (std::size_t i = 0 ; i < robotParts_.size(); ++i)
		{
			poseFromStateCallback_(pos, rot, extractState_(s1, i));
			transi_beg.setTransform(rot, pos);
			poseFromStateCallback_(pos, rot, extractState_(s2, i));
			transi_end.setTransform(rot, pos);

			for (std::size_t j = i+1; j < robotParts_.size(); ++j)
			{
				poseFromStateCallback_(pos, rot, extractState_(s1, j));
				transj_beg.setTransform(rot, pos);
				poseFromStateCallback_(pos, rot, extractState_(s2, j));
				transj_end.setTransform(rot, pos);

				// Checking for collision
				fcl::continuousCollide(robotParts_[i], transi_beg, transi_end,
						robotParts_[j], transj_beg, transj_end,
						collisionRequest, collisionResult);
				if (collisionResult.is_collide)
				{
					collisionTime = collisionResult.time_of_contact;
					return false;
				}
			}
		}
	}
	return true;
}

/// \brief Returns the minimum distance from the given robot state and the environment
virtual double clearance (const base::State *state) const
{
	double minDist = std::numeric_limits<double>::infinity ();
	if (environment_.num_tris > 0)
	{
		fcl::DistanceRequest distanceRequest(true);
		fcl::DistanceResult distanceResult;
		fcl::Transform3f trans, trans_env;
		fcl::Quaternion3f rot;
		fcl::Vec3f pos;
		fcl::MeshDistanceTraversalNodeOBBRSS distanceNode;
		for (size_t i = 0; i < robotParts_.size (); ++i)
		{
			poseFromStateCallback_(pos, rot, extractState_(state, i));
			trans.setTransform(rot, pos);
			fcl::initialize(distanceNode, *robotParts_[i], trans, environment_, trans_env, distanceRequest, distanceResult);
			fcl::distance (&distanceNode);
			if (distanceResult.min_distance < minDist)
				minDist = distanceResult.min_distance;
		}
	}

	return minDist;
}*/

