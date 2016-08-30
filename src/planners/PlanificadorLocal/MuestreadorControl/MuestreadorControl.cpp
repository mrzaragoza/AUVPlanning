#include "planners/PlanificadorLocal/MuestreadorControl/MuestreadorControl.h"

ompl::base::State  *ompl::guillermo::MuestreadorControl::goal_ = NULL;
ompl::base::State  *ompl::guillermo::MuestreadorControl::start_ = NULL;


ompl::guillermo::MuestreadorControl::MuestreadorControl(const ompl::control::ControlSpace *space) : control::ControlSampler::ControlSampler(space),space_mc(space)
{
    /*goal_->as<base::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[2] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[3] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[4] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[5] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[6] = 0.0;
    goal_->as<base::RealVectorStateSpace::StateType>()->values[7] = 0.0;*/

    printf("MC constructor!\n");
    fflush(stdout);
    cont = 0;
    controlZEstable = 0.49699768; //En MATLAB: 0.1988;
    tiempoBase      = 50;
    maxDiferenciaControl = 0.5;
    margenCercania = 0.2;
}

void ompl::guillermo::MuestreadorControl::sample(control::Control *control)
{
	printf("MC ergsergsgeaertaertgaergergergaert!\n");
    fflush(stdout);
    const unsigned int dim = space_mc->getDimension();

    if(dim != 3){
    	printf("ompl::guillermo::MuestreadorControl::sample: Error! El numero de dimensiones no concuerda. Dim = %d\n",dim);
    }

    const base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(space_mc)->getBounds();

    ompl::control::RealVectorControlSpace::ControlType *newControl = static_cast<ompl::control::RealVectorControlSpace::ControlType*>(control);

    for (unsigned int i = 0 ; i < dim ; ++i)
        //newControl->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);/////////////////
        newControl->values[i] = std::rand()*2-1;/////////////////
}

void ompl::guillermo::MuestreadorControl::sample(control::Control *control, const base::State * state)
{

	//printf("MC sample Inicia!\n");
    //fflush(stdout);
    /*std::vector<int> signature;
    space_mc->computeSignature(signature);

    printf("Space_MC signature: ");
    fflush(stdout);
    for(int i=0; i<signature.size();i++){
        printf("%d ",signature[i]);
        fflush(stdout);
    }
    printf("\n");
    fflush(stdout);*/
    const unsigned int dim =3;
    /*const unsigned int dim = space_mc->getDimension();

    if(dim != 3){
    	printf("ompl::guillermo::MuestreadorControl::sample: Error! El numero de dimensiones no concuerda. Dim = %d\n",dim);
    }*/

    //printf("MC sample 1!\n");
    //fflush(stdout);

    //const base::RealVectorBounds &bounds = static_cast<const ompl::control::RealVectorControlSpace*>(space_mc)->getBounds();

    //printf("MC sample 2!\n");
    //fflush(stdout);

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

	//printf("MC sample 3!\n");
    //fflush(stdout);

	//Normalización entre 0 y 1 de la distancia actual respecto a la inicial
	double norm_distancia 		= 0;
	double norm_distancia_3D 	= 0;
	double norm_distancia_z 	= 0;

	if(dist_total != 0) 	norm_distancia 		= dist_actual / dist_total;

	if(dist_total_3D != 0) 	norm_distancia_3D 	= dist_actual_3D / dist_total_3D;

	if(diff_z_inicial != 0) norm_distancia_z 	= diff_z / diff_z_inicial;

	/*printf("MC norm_distancia: %f norm_distancia_3D: %f norm_distancia_z: %f\n",norm_distancia,norm_distancia_3D,norm_distancia_z);
    fflush(stdout);*/
	if(norm_distancia > 1) norm_distancia = 1;
	if(norm_distancia_3D > 1) norm_distancia_3D = 1;
	if(norm_distancia_z > 1) norm_distancia_z = 1;

	/*printf("MC norm_distancia: %f norm_distancia_3D: %f norm_distancia_z: %f\n",norm_distancia,norm_distancia_3D,norm_distancia_z);
    fflush(stdout);*/

	//printf("MC sample 4!\n");
    //fflush(stdout);

	//Calculo del ángulo que hay entre el estado actual y el final y la diferencia entre éste y el yaw del vehículo
	double heading = atan2(diff_y,diff_x); //radianes
	double diff_heading = heading - state->as<base::RealVectorStateSpace::StateType>()->values[3];

	//double num_aleatorio = rng_.uniformReal(0,1);/////////////////
	double num_aleatorio = std::rand();/////////////////
	double aux = 0;
	double bh = 0;
	if(diff_heading > margenCercania){
		bh = 1;//bh = bounds.high[1];
		//aux = rng_.uniformReal(0, bh);/////////////////
		aux = std::rand();/////////////////
	    double a = num_aleatorio + aux*maxDiferenciaControl/2;
	    newControl->values[1] = a;
	    //newControl->values[2] = num_aleatorio - rng_.uniformReal(0, bounds.high[2])*maxDiferenciaControl/2;/////////////////
	    newControl->values[2] = num_aleatorio - std::rand()*maxDiferenciaControl/2;/////////////////
	}else if(diff_heading < -margenCercania){
		bh = 1;//bh = bounds.high[1];
		//aux = rng_.uniformReal(0, bh);/////////////////
		aux = std::rand();/////////////////
	    double b = num_aleatorio - aux*maxDiferenciaControl/2;
	    newControl->values[1] = b;
	    //newControl->values[2] = num_aleatorio + rng_.uniformReal(0, bounds.high[2])*maxDiferenciaControl/2;/////////////////
	    newControl->values[2] = num_aleatorio + std::rand()*maxDiferenciaControl/2;/////////////////
	}else{ //diff_heading ~ 0
		double bl = -1;//bounds.low[1];
		bh = 1;//bh = bounds.high[1];
		//aux = rng_.uniformReal(bl, bh);/////////////////
		aux = std::rand();/////////////////
	    double c = num_aleatorio + aux*maxDiferenciaControl/8;
	    newControl->values[1] = c;
	    //newControl->values[2] = num_aleatorio + rng_.uniformReal(bounds.low[2], bounds.high[2])*maxDiferenciaControl/8;/////////////////
	    newControl->values[2] = num_aleatorio + (std::rand()*2-1)*maxDiferenciaControl/8;/////////////////
	}

	if(diff_z > margenCercania){
	    //newControl->values[0] = rng_.uniformReal(0, bounds.high[0]);/////////////////
	    newControl->values[0] = std::rand();/////////////////
	}else if(diff_z < -margenCercania){
	    //newControl->values[0] = rng_.uniformReal(bounds.low[0], 0);/////////////////
	    newControl->values[0] = std::rand()-1;/////////////////
	}else{ //diff_z ~ 0
	    //newControl->values[0] = rng_.uniformReal(bounds.low[0], bounds.high[0]);/////////////////
	    newControl->values[0] = std::rand()*2-1;/////////////////
	    newControl->values[0] = newControl->values[0] * 0.1 * controlZEstable;
	}

	//Se multiplica por la normalización de la distancia, haciendo que cuanto
	//más cerca se esté del objetivo, los controles sean más pequeños.

	/*printf("MC T1: %f T2: %f T3: %f  Antes de normalización\n",newControl->values[0],newControl->values[1],newControl->values[2]);
    fflush(stdout);*/

	if(norm_distancia_z != 0 ) newControl->values[0] = newControl->values[0] * norm_distancia_z;
		    
	newControl->values[1] = newControl->values[1] * norm_distancia;
	newControl->values[2] = newControl->values[2] * norm_distancia;

	/*printf("MC T1: %f T2: %f T3: %f\n",newControl->values[0],newControl->values[1],newControl->values[2]);
    fflush(stdout);*/

	if(newControl->values[0] > 1) newControl->values[0] = 1;
	else if(newControl->values[0] < -1) newControl->values[0] = -1;
	
	if(newControl->values[1] > 1) newControl->values[1] = 1;
	else if(newControl->values[1] < -1) newControl->values[1] = -1;
	
	if(newControl->values[2] > 1) newControl->values[2] = 1;
	else if(newControl->values[2] < -1) newControl->values[2] = -1;

	cont++;
	printf("MC CONT: %d\n",cont);
    fflush(stdout);
	
}

void ompl::guillermo::MuestreadorControl::sampleNext(control::Control *control, const control::Control * /* previous */)
{
	printf("MC ababbababababababababababababababa!\n");
    fflush(stdout);
    sample(control);
}

void ompl::guillermo::MuestreadorControl::sampleNext(control::Control *control, const control::Control * /* previous */, const base::State * state)
{
	printf("MC babababababababababababab!\n");
    fflush(stdout);
    sample(control,state);
}

unsigned int ompl::guillermo::MuestreadorControl::sampleStepCount(unsigned int minSteps, unsigned int maxSteps)
{
    //return rng_.uniformInt(minSteps, maxSteps);/////////////////
    return std::rand()%(maxSteps-minSteps+1)+minSteps;/////////////////
}

void ompl::guillermo::MuestreadorControl::setGoal(base::State *state)
{
	printf("MC setGoal!\n");
    fflush(stdout);

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

ompl::base::State* ompl::guillermo::MuestreadorControl::getGoal()
{
	printf("MC getGoal!\n");
    fflush(stdout);
    return goal_;
}

void ompl::guillermo::MuestreadorControl::setStart(base::State *state)
{
	printf("MC setStart!\n");
    fflush(stdout);
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

ompl::base::State* ompl::guillermo::MuestreadorControl::getStart()
{
	printf("MC getStart!\n");
    fflush(stdout);
    return start_;
}