/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "planners/DynamicRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::auvplanning::DynamicRRT::DynamicRRT(const control::SpaceInformationPtr &si) : base::Planner(si, "DynamicRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    lastGoalMotion_ = NULL;

    goalBias_ = 0.25;

    Planner::declareParam<double>("goal_bias", this, &DynamicRRT::setGoalBias, &DynamicRRT::getGoalBias, "0.:.25:1.");
}

ompl::auvplanning::DynamicRRT::~DynamicRRT()
{
    freeMemory();
}

void ompl::auvplanning::DynamicRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    nn_->setDistanceFunction(boost::bind(&DynamicRRT::distanceFunction, this, _1, _2));
}

void ompl::auvplanning::DynamicRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::auvplanning::DynamicRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->currentState)
                si_->freeState(motions[i]->currentState);
            if (motions[i]->referenceState)
                si_->freeState(motions[i]->referenceState);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus ompl::auvplanning::DynamicRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->currentState, st);
        //si_->copyState(motion->referenceState, NULL); //No hace falta, pues es un estado inicial, por lo que no tiene referencia.
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();

    Motion      			*rmotion = new Motion(siC_);
    base::State  			*rstate = rmotion->currentState;
    base::State       		*rfinalState = rmotion->referenceState;
    control::Control			*rctrl = siC_->allocControl();
    int contador = 1;

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        //if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        if(contador%1 == 0)
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        base::State *intermediate_state = si_->allocState();
        si_->copyState(intermediate_state, nmotion->currentState);

        base::State *intermediate_final_state = si_->allocState();
        
        //Total distance
        double tdist = si_->distance(nmotion->currentState, rmotion->currentState);
        /*int max_num_intermediate_states = 10;
        int num_intermediate_states = 0;*/
        unsigned int tiempo = 0;

        while(si_->distance(intermediate_state, rmotion->currentState) > 0.2*tdist /*&& num_intermediate_states < max_num_intermediate_states*/){
        	//printf("hola\n");
        	si_->copyState(intermediate_final_state, rstate);
        	/*printf("Objetivo. intermediate_final_state -> %f, %f, %f, %f\n", intermediate_final_state->as<base::RealVectorStateSpace::StateType>()->values[0],
				intermediate_final_state->as<base::RealVectorStateSpace::StateType>()->values[1],intermediate_final_state->as<base::RealVectorStateSpace::StateType>()->values[2],
				intermediate_final_state->as<base::RealVectorStateSpace::StateType>()->values[3]);*/
        	unsigned int t = controlSampler_->sampleTo(rctrl, intermediate_state, intermediate_final_state);
        	//printf("adios. tiempo = %d t = %d minimo = %d maximo = %d\n", tiempo, t, siC_->getMinControlDuration(), siC_->getMaxControlDuration());
        	si_->copyState(intermediate_state, intermediate_final_state);

        	tiempo = tiempo + t;

        	if (tiempo > siC_->getMaxControlDuration() || t < siC_->getMinControlDuration()) break;
        	//printf("adios 2\n");
        	/* create a motion */
            Motion *motion = new Motion(siC_);
            si_->copyState(motion->currentState, intermediate_state);
            si_->copyState(motion->referenceState, rmotion->currentState);
            motion->steps = tiempo;
            motion->parent = nmotion;

            nn_->add(motion);

            double dist = 0.0;
            if(goal->isSatisfied(motion->currentState, &dist)){ 
                approxdif = dist;
                solution = motion;
            	break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        	//num_intermediate_states++;
        }

        si_->freeState(intermediate_state);
        si_->freeState(intermediate_final_state);

        contador++;
    }

    bool solved = false;
    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathController *path = new PathController(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i){
        	printf("A\n");
            if (mpath[i]->parent)
                path->append(mpath[i]->currentState, mpath[i]->referenceState, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->currentState);
        }
        solved = true;
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
    }

    if (rmotion->currentState)
        si_->freeState(rmotion->currentState);
    if (rmotion->referenceState)
        si_->freeState(rmotion->referenceState);
    if(rctrl)
    	siC_->freeControl(rctrl);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::auvplanning::DynamicRRT::getPlannerData(base::PlannerData &data) const
{
    OMPL_INFORM("%s: DynamicRRT::getPlannerData called", getName().c_str());
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->currentState));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        const Motion *m = motions[i];
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->currentState),
                             base::PlannerDataVertex(m->currentState),
                             control::PlannerDataEdgeControl(NULL, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->currentState),
                             base::PlannerDataVertex(m->currentState));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->currentState));
    }
    OMPL_INFORM("%s: DynamicRRT::getPlannerData exit", getName().c_str());
}