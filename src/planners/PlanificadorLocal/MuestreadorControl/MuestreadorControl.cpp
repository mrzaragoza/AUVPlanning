#include "planners/PlanificadorLocal/MuestreadorControl/MuestreadorControl.h"


ompl::guillermo::MuestreadorControl::MuestreadorControl(const ompl::control::ControlSpace *space) : control::ControlSampler::ControlSampler(space)
{
    goal_->as<base::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[2] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[3] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[4] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[5] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[6] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[7] = 0.0;

    controlZEstable = 0.07;
    tiempoBase      = 50;
    maxDiferenciaControl = 0.5;
    margenCercania = 0.2;
}

void ompl::guillermo::MuestreadorControl::sample(control::Control *control)
{
    const unsigned int dim = space_->getDimension();

    if(dim != 3){
    	printf("ompl::guillermo::MuestreadorControl::sample: Error! El numero de dimensiones no concuerda. Dim = %d\n",dim);
    }

    const base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(space_)->getBounds();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::RealVectorControlSpace::ControlType*>(control);

    for (unsigned int i = 0 ; i < dim ; ++i)
        newControl->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}

void ompl::guillermo::MuestreadorControl::sample(control::Control *control, const base::State * state)
{
    const unsigned int dim = space_->getDimension();

    if(dim != 3){
    	printf("ompl::guillermo::MuestreadorControl::sample: Error! El numero de dimensiones no concuerda. Dim = %d\n",dim);
    }

    const base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(space_)->getBounds();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::RealVectorControlSpace::ControlType*>(control);

    //Distancias con estado inicial
	double diff_x_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[0]-state->as<base::RealVectorStateSpace::StateType>()->values[0];
	double diff_y_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[1]-state->as<base::RealVectorStateSpace::StateType>()->values[1];
	double diff_z_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[2]-state->as<base::RealVectorStateSpace::StateType>()->values[2];
	double dist_total 			= sqrt(pow(diff_x_inicial,2) + pow(diff_y_inicial,2) );
	double dist_total_3D 		= sqrt(pow(diff_x_inicial,2) + pow(diff_y_inicial,2) + pow(diff_z_inicial,2));

	//Distancias con estado actual
	double diff_x 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[0]-state->as<base::RealVectorStateSpace::StateType>()->values[0];
	double diff_y 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[1]-state->as<base::RealVectorStateSpace::StateType>()->values[1];
	double diff_z 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[2]-state->as<base::RealVectorStateSpace::StateType>()->values[2];
	double dist_actual 			= sqrt(pow(diff_x,2) + pow(diff_y,2) );
	double dist_actual_3D 		= sqrt(pow(diff_x,2) + pow(diff_y,2) + pow(diff_z,2));

	//Normalización entre 0 y 1 de la distancia actual respecto a la inicial
	double norm_distancia 		= 0;
	double norm_distancia_3D 	= 0;
	double norm_distancia_z 	= 0;

	if(dist_total != 0) 	norm_distancia 		= dist_actual / dist_total;

	if(dist_total_3D != 0) 	norm_distancia_3D 	= dist_actual_3D / dist_total_3D;

	if(diff_z_inicial != 0) norm_distancia_z 	= diff_z / diff_z_inicial;


	//Calculo del ángulo que hay entre el estado actual y el final y la diferencia entre éste y el yaw del vehículo
	double heading = atan2(diff_y,diff_x); //radianes
	double diff_heading = heading - state->as<base::RealVectorStateSpace::StateType>()->values[3];

	double num_aleatorio = rng_.uniformReal(0,1);
	if(diff_heading > margenCercania){
	    newControl->values[1] = num_aleatorio + rng_.uniformReal(0, bounds.high[1])*maxDiferenciaControl/2;
	    newControl->values[2] = num_aleatorio - rng_.uniformReal(0, bounds.high[2])*maxDiferenciaControl/2;
	}else if(diff_heading < -margenCercania){
	    newControl->values[1] = num_aleatorio - rng_.uniformReal(0, bounds.high[1])*maxDiferenciaControl/2;
	    newControl->values[2] = num_aleatorio + rng_.uniformReal(0, bounds.high[2])*maxDiferenciaControl/2;
	}else{ //diff_heading ~ 0
	    newControl->values[1] = num_aleatorio + rng_.uniformReal(bounds.low[1], bounds.high[1])*maxDiferenciaControl/8;
	    newControl->values[2] = num_aleatorio + rng_.uniformReal(bounds.low[2], bounds.high[2])*maxDiferenciaControl/8;
	}

	if(diff_z > margenCercania){
	    newControl->values[0] = rng_.uniformReal(0, bounds.high[0]);
	}else if(diff_z < -margenCercania){
	    newControl->values[0] = rng_.uniformReal(bounds.low[0], 0);
	}else{ //diff_z ~ 0
	    newControl->values[0] = rng_.uniformReal(bounds.low[0], bounds.high[0]);
	    newControl->values[0] = newControl->values[0] * 0.1 * controlZEstable;
	}

	//Se multiplica por la normalización de la distancia, haciendo que cuanto
	//más cerca se esté del objetivo, los controles sean más pequeños.

	if(norm_distancia_z != 0 ) newControl->values[0] = newControl->values[0] * norm_distancia_z;
		    
	newControl->values[1] = newControl->values[1] * norm_distancia;
	newControl->values[2] = newControl->values[2] * norm_distancia;

	if(newControl->values[0] > 1) newControl->values[0] = 1;
	else if(newControl->values[0] < -1) newControl->values[0] = -1;
	
	if(newControl->values[1] > 1) newControl->values[1] = 1;
	else if(newControl->values[1] < -1) newControl->values[1] = -1;
	
	if(newControl->values[2] > 1) newControl->values[2] = 1;
	else if(newControl->values[2] < -1) newControl->values[2] = -1;
	
}

void ompl::guillermo::MuestreadorControl::sampleNext(control::Control *control, const control::Control * /* previous */)
{
    sample(control);
}

void ompl::guillermo::MuestreadorControl::sampleNext(control::Control *control, const control::Control * /* previous */, const base::State * state)
{
    sample(control,state);
}

unsigned int ompl::guillermo::MuestreadorControl::sampleStepCount(unsigned int minSteps, unsigned int maxSteps)
{
    return rng_.uniformInt(minSteps, maxSteps);
}

void ompl::guillermo::MuestreadorControl::setGoal(base::State *state)
{
    goal_ = state;
}

ompl::base::State* ompl::guillermo::MuestreadorControl::getGoal()
{
    return goal_;
}