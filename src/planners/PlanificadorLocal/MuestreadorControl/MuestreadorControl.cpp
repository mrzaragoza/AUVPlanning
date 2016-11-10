#include "planners/PlanificadorLocal/MuestreadorControl/MuestreadorControl.h"

ompl::base::State  *ompl::auvplanning::MuestreadorControl::goal_ = NULL;
ompl::base::State  *ompl::auvplanning::MuestreadorControl::start_ = NULL;


ompl::auvplanning::MuestreadorControl::MuestreadorControl(const ompl::control::ControlSpace *space) : control::ControlSampler::ControlSampler(space),space_mc(space),
controlZEstable(0.49699768),tiempoBase(50),maxDiferenciaControl(0.5),margenCercania(0.2)
{
    //cont = 0;
    controlZEstable = 0.49699768; //En MATLAB: 0.1988;
    tiempoBase      = 50;
    maxDiferenciaControl = 0.5;
    margenCercania = 0.2;

}

void ompl::auvplanning::MuestreadorControl::sample(control::Control *control)
{
	printf("MC ergsergsgeaertaertgaergergergaert!\n");
    fflush(stdout);
    const unsigned int dim = space_mc->getDimension();

    if(dim != 3){
    	printf("ompl::auvplanning::MuestreadorControl::sample: Error! El numero de dimensiones no concuerda. Dim = %d\n",dim);
    }

    const base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(space_mc)->getBounds();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::RealVectorControlSpace::ControlType*>(control);

    for (unsigned int i = 0 ; i < dim ; ++i)
        //newControl->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);/////////////////
        newControl->values[i] = ((double) std::rand() / RAND_MAX)*2-1;/////////////////
}

void ompl::auvplanning::MuestreadorControl::sample(control::Control *control, const base::State * state)
{

    const unsigned int dim =3;

    const base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(space_mc)->getBounds();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::RealVectorControlSpace::ControlType*>(control);

	//Distancias con estado actual
	double diff_x 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[0] - 
										state->as<base::RealVectorStateSpace::StateType>()->values[0];
	double diff_y 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[1] - 
										state->as<base::RealVectorStateSpace::StateType>()->values[1];
	double diff_z 				= goal_->as<base::RealVectorStateSpace::StateType>()->values[2] - 
										state->as<base::RealVectorStateSpace::StateType>()->values[2];
	double dist_actual 			= sqrt(pow(diff_x,2) + pow(diff_y,2) );
	double dist_actual_3D 		= sqrt(pow(diff_x,2) + pow(diff_y,2) + pow(diff_z,2));


    controlZEstable = 0.49699768; //En MATLAB: 0.1988;
    tiempoBase      = 50;
    maxDiferenciaControl = 0.5;
    margenCercania = 0.2;

	/*printf("***************** goal_->as<base::RealVectorStateSpace::StateType>()->values[0]: %f\n", goal_->as<base::RealVectorStateSpace::StateType>()->values[0]);
	printf("***************** state->as<base::RealVectorStateSpace::StateType>()->values[0]: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[0]);
	printf("***************** goal_->as<base::RealVectorStateSpace::StateType>()->values[1]: %f\n", goal_->as<base::RealVectorStateSpace::StateType>()->values[1]);
	printf("***************** state->as<base::RealVectorStateSpace::StateType>()->values[1]: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[1]);



	printf("***************** diff_x: %f\n", diff_x);
	printf("***************** diff_y: %f\n", diff_y);

	printf("***************** dist_actual: %f\n", dist_actual);
	printf("***************** dist_total: %f\n", dist_total);

	printf("***************** dist_actual_3D: %f\n", dist_actual_3D);
	printf("***************** dist_total_3D: %f\n", dist_total_3D);*/


	//Normalización entre 0 y 1 de la distancia actual respecto a la inicial
	double norm_distancia 		= 0;
	double norm_distancia_3D 	= 0;
	double norm_distancia_z 	= 0;

	if(dist_total != 0) 	norm_distancia 		= dist_actual / dist_total;

	if(dist_total_3D != 0) 	norm_distancia_3D 	= dist_actual_3D / dist_total_3D;

	if(diff_z_inicial != 0) norm_distancia_z 	= diff_z / diff_z_inicial;

	/*printf("*****************1 norm_distancia: %f\n", norm_distancia);
	printf("*****************1 norm_distancia_3D: %f\n", norm_distancia_3D);
	printf("*****************1 norm_distancia_z: %f\n", norm_distancia_z);*/

	if(norm_distancia > 1) norm_distancia = 1;
	if(norm_distancia_3D > 1) norm_distancia_3D = 1;
	if(norm_distancia_z > 1) norm_distancia_z = 1;

	/*printf("*****************2 norm_distancia: %f\n", norm_distancia);
	printf("*****************2 norm_distancia_3D: %f\n", norm_distancia_3D);
	printf("*****************2 norm_distancia_z: %f\n", norm_distancia_z);*/

	//Calculo del ángulo que hay entre el estado actual y el final y la diferencia entre éste y el yaw del vehículo
	double heading = atan2(diff_y,diff_x); //radianes
	double diff_heading = heading - state->as<base::RealVectorStateSpace::StateType>()->values[3];

	//double num_aleatorio = rng_.uniformReal(0,1);/////////////////
	double num_aleatorio = (double) std::rand() / RAND_MAX;/////////////////
	/*printf("*****************num aleatorio: %f\n", num_aleatorio);
	printf("*****************maxDiferenciaControl %f\n", maxDiferenciaControl);
    printf("*****************controlZEstable %f\n", controlZEstable);
    printf("*****************tiempoBase %d\n", tiempoBase);
    printf("*****************margenCercania %f\n", margenCercania);*/
	if(diff_heading > margenCercania){
	    //newControl->values[1] = num_aleatorio + rng_.uniformReal(0, bounds.high[1])*maxDiferenciaControl/2;/////////////////
	    double aw= num_aleatorio + ((double) std::rand() / RAND_MAX)*maxDiferenciaControl/2;
	    newControl->values[1] = aw;/////////////////
	    //newControl->values[2] = num_aleatorio - rng_.uniformReal(0, bounds.high[2])*maxDiferenciaControl/2;/////////////////
	    double ay= num_aleatorio - ((double) std::rand() / RAND_MAX)*maxDiferenciaControl/2;
	    newControl->values[2] = ay;/////////////////

	}else if(diff_heading < -margenCercania){		
	    //newControl->values[1] = num_aleatorio - rng_.uniformReal(0, bounds.high[1])*maxDiferenciaControl/2;/////////////////
	    double aq= num_aleatorio - ((double) std::rand() / RAND_MAX)*maxDiferenciaControl/2;
	    newControl->values[1] = aq;/////////////////
	    //newControl->values[2] = num_aleatorio + rng_.uniformReal(0, bounds.high[2])*maxDiferenciaControl/2;/////////////////
	    double as= num_aleatorio + ((double) std::rand() / RAND_MAX)*maxDiferenciaControl/2;
	    newControl->values[2] = as;/////////////////

	}else{ //diff_heading ~ 0
	    //newControl->values[1] = num_aleatorio + rng_.uniformReal(bounds.low[1], bounds.high[1])*maxDiferenciaControl/8;/////////////////
	    double ad= num_aleatorio + (((double) std::rand() / RAND_MAX)*2-1)*maxDiferenciaControl/8;
	    newControl->values[1] = ad;/////////////////
	    //newControl->values[2] = num_aleatorio + rng_.uniformReal(bounds.low[2], bounds.high[2])*maxDiferenciaControl/8;/////////////////
	    double af= num_aleatorio + (((double) std::rand() / RAND_MAX)*2-1)*maxDiferenciaControl/8;
	    newControl->values[2] = af;/////////////////
	}

	if(diff_z > margenCercania){
	    //newControl->values[0] = rng_.uniformReal(0, bounds.high[0]);/////////////////
	    double ax = ((double) std::rand() / RAND_MAX);
	    newControl->values[0] = ax;/////////////////

	}else if(diff_z < -margenCercania){
	    //newControl->values[0] = rng_.uniformReal(bounds.low[0], 0);/////////////////
	    double ax = ((double) std::rand() / RAND_MAX)-1;
	    newControl->values[0] = ax;/////////////////

	}else{ //diff_z ~ 0
	    //newControl->values[0] = rng_.uniformReal(bounds.low[0], bounds.high[0]);/////////////////
	    double ax = (((double) std::rand() / RAND_MAX)*2-1) * 0.1 + controlZEstable;
	    newControl->values[0] = ax;/////////////////
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

	/*cont++;
	printf("MC CONT: %d\n",cont);
    fflush(stdout);*/
	
}

void ompl::auvplanning::MuestreadorControl::sampleNext(control::Control *control, const control::Control * /* previous */)
{
	printf("MC ababbababababababababababababababa!\n");
    fflush(stdout);
    sample(control);
}

void ompl::auvplanning::MuestreadorControl::sampleNext(control::Control *control, const control::Control * /* previous */, const base::State * state)
{
	printf("MC babababababababababababab!\n");
    fflush(stdout);
    sample(control,state);
}

unsigned int ompl::auvplanning::MuestreadorControl::sampleStepCount(unsigned int minSteps, unsigned int maxSteps)
{
    //return rng_.uniformInt(minSteps, maxSteps);/////////////////
    return  std::rand() %(maxSteps-minSteps+1)+minSteps;/////////////////
    //return  (maxSteps+minSteps)/2;/////////////////
}

void ompl::auvplanning::MuestreadorControl::setGoal(base::State *state)
{
	/*printf("MC setGoal!\n");
    fflush(stdout);*/

    goal_ = state;

    if(start_ != NULL){
	    //Distancias con estado inicial
		diff_x_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[0] - 
											start_->as<base::RealVectorStateSpace::StateType>()->values[0];
		diff_y_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[1] - 
											start_->as<base::RealVectorStateSpace::StateType>()->values[1];
		diff_z_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[2] - 
											start_->as<base::RealVectorStateSpace::StateType>()->values[2];
		dist_total 			= sqrt(pow(diff_x_inicial,2) + pow(diff_y_inicial,2) );
		dist_total_3D 		= sqrt(pow(diff_x_inicial,2) + pow(diff_y_inicial,2) + pow(diff_z_inicial,2));
	}
}

ompl::base::State* ompl::auvplanning::MuestreadorControl::getGoal()
{
	/*printf("MC getGoal!\n");
    fflush(stdout);*/
    return goal_;
}

void ompl::auvplanning::MuestreadorControl::setStart(base::State *state)
{
	/*printf("MC setStart!\n");
    fflush(stdout);*/
    start_ = state;


    if(goal_ != NULL){
	    //Distancias con estado inicial
    	double a = goal_->as<base::RealVectorStateSpace::StateType>()->values[0];
    	double b = start_->as<base::RealVectorStateSpace::StateType>()->values[0];

		diff_x_inicial 		=  a- b;
		diff_y_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[1] - 
											start_->as<base::RealVectorStateSpace::StateType>()->values[1];
		diff_z_inicial 		= goal_->as<base::RealVectorStateSpace::StateType>()->values[2] - 
											start_->as<base::RealVectorStateSpace::StateType>()->values[2];
		dist_total 			= sqrt(pow(diff_x_inicial,2) + pow(diff_y_inicial,2) );
		dist_total_3D 		= sqrt(pow(diff_x_inicial,2) + pow(diff_y_inicial,2) + pow(diff_z_inicial,2));
	}
}

ompl::base::State* ompl::auvplanning::MuestreadorControl::getStart()
{
	/*printf("MC getStart!\n");
    fflush(stdout);*/
    return start_;
}