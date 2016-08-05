#include "planners/PlanificadorLocal/MuestreadorControl/MuestreadorControl.h"


void ompl::guillermo::MuestreadorControl::sample(Control *control)
{
    const unsigned int dim = space_->getDimension();

    if(dim != 3){
    	printf("ompl::guillermo::MuestreadorControl::sample: Error! El numero de dimensiones no concuerda. Dim = %d\n",dim);
    }

    const base::RealVectorBounds &bounds = static_cast<const RealVectorControlSpace*>(space_)->getBounds();

    RealVectorControlSpace::ControlType *newControl = static_cast<RealVectorControlSpace::ControlType*>(control);

    for (unsigned int i = 0 ; i < dim ; ++i)
        newControl->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}

void ompl::guillermo::MuestreadorControl::sample(Control *control, const base::State * state)
{
    const unsigned int dim = space_->getDimension();

    if(dim != 3){
    	printf("ompl::guillermo::MuestreadorControl::sample: Error! El numero de dimensiones no concuerda. Dim = %d\n",dim);
    }

    const base::RealVectorBounds &bounds = static_cast<const RealVectorControlSpace*>(space_)->getBounds();

    RealVectorControlSpace::ControlType *newControl = static_cast<RealVectorControlSpace::ControlType*>(control);

    //Distancias con estado inicial
	double diff_x_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[0]-state->as<base::RealVectorStateSpace::StateType>()->values[0];
	double diff_y_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[1]-state->as<base::RealVectorStateSpace::StateType>()->values[1];
	double diff_z_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[2]-state->as<base::RealVectorStateSpace::StateType>()->values[2];
	double dist_total 			= sqrt(diff_x_inicial^ 2 + diff_y_inicial^2 );
	double dist_total_3D 		= sqrt(diff_x_inicial^ 2 + diff_y_inicial^2 + diff_z_inicial^2 );

	//Distancias con estado actual
	double diff_x 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[0]-state->as<base::RealVectorStateSpace::StateType>()->values[0];
	double diff_y 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[1]-state->as<base::RealVectorStateSpace::StateType>()->values[1];
	double diff_z 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[2]-state->as<base::RealVectorStateSpace::StateType>()->values[2];
	double dist_actual 			= sqrt(diff_x^ 2 + diff_y^2 );
	double dist_actual_3D 		= sqrt(diff_x^ 2 + diff_y^2 + diff_z^2);

	//Normalización entre 0 y 1 de la distancia actual respecto a la inicial
	double norm_distancia 		= 0;
	double norm_distancia_3D 	= 0;
	double norm_distancia_z 	= 0;

	if(dist_total != 0) 	norm_distancia 		= dist_actual / dist_total;

	if(dist_total_3D != 0) 	norm_distancia_3D 	= dist_actual_3D / dist_total_3D;

	if(diff_z_inicial != 0) norm_distancia_z 	= diff_z / diff_z_inicial;


	//Calculo del ángulo que hay entre el estado actual y el final y la diferencia entre éste y el yaw del vehículo
	double heading = atan2(diff_y,diff_x) //radianes
	double diff_heading = heading - st_actual[3];

	num_aleatorio = rand(1);
	if(diff_heading > margen_cercania){
	    newControl->values[1] = num_aleatorio + rand(1)*max_diff_thrusters/2;
	    newControl->values[2] = num_aleatorio - rand(1)*max_diff_thrusters/2;
	}else if(diff_heading < -margen_cercania){
	    newControl->values[1] = num_aleatorio - rand(1)*max_diff_thrusters/2;
	    newControl->values[2] = num_aleatorio + rand(1)*max_diff_thrusters/2;
	}else{ //diff_heading ~ 0
	    newControl->values[1] = num_aleatorio + (rand(1)*2 -1)*max_diff_thrusters/8;
	    newControl->values[2] = num_aleatorio + (rand(1)*2 -1)*max_diff_thrusters/8;
	}

	if(diff_z > margen_cercania){
	    newControl->values[0] = rand(1);
	}else if(diff_z < -margen_cercania){
	    newControl->values[0] = rand(1)*(-1);
	}else{ //diff_z ~ 0
	    newControl->values[0] = (rand(1)*2 - 1);
	    newControl->values[0] = newControl->values[0] * 0.1 * control_z_estable;
	}

	//Se multiplica por la normalización de la distancia, haciendo que cuanto
	//más cerca se esté del objetivo, los controles sean más pequeños.

	if(norm_distancia_z ~= 0 ) control(1) = control(1) * norm_distancia_z;
		    
	control(2) = control(2) * norm_distancia;
	control(3) = control(3) * norm_distancia;

	if(control(1)>1) control(1) = 1;
	elseif(control(1)<-1) control(1) = -1;
	
	if(control(2)>1) control(2) = 1;
	elseif(control(2)<-1) control(2) = -1;
	
	if(control(3)>1) control(3) = 1;
	elseif(control(3)<-1) control(3) = -1;
	


    for (unsigned int i = 0 ; i < dim ; ++i)
        newControl->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}

void ompl::guillermo::MuestreadorControl::sampleNext(Control *control, const Control * /* previous */)
{
    sample(control);
}

void ompl::guillermo::MuestreadorControl::sampleNext(Control *control, const Control * /* previous */, const base::State * state)
{
    sample(control,state);
}

unsigned int ompl::guillermo::MuestreadorControl::sampleStepCount(unsigned int minSteps, unsigned int maxSteps)
{
    return rng_.uniformInt(minSteps, maxSteps);
}

void ompl::guillermo::MuestreadorControl::setGoal(const base::State state)
{
    goal_ = state;
}

void ompl::guillermo::MuestreadorControl::getGoal()
{
    return goal_;
}