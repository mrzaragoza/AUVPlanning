#include "benchmarks/AUVPlanning.h"

using namespace ompl;
namespace oauv = ompl::auvplanning;

#define DCS_TYPE  AUV_PID_DCS

void AUVRobotSetup(oauv::AUVRobotPtr& robot, YAML::Node config)
{
	base::StateSpacePtr StSpace(robot->getSimpleSetup().getStateSpace());

	// define start state
	base::ScopedState<base::RealVectorStateSpace> start(robot->getGeometricComponentStateSpace());
	start[0] = config["general/start"][0].as<double>();
	start[1] = config["general/start"][1].as<double>();
	start[2] = config["general/start"][2].as<double>();	

	// define goal state
    base::ScopedState<base::RealVectorStateSpace> goal(robot->getGeometricComponentStateSpace());
    goal[0] = config["general/goal"][0].as<double>();
    goal[1] = config["general/goal"][1].as<double>();
    goal[2] = config["general/goal"][2].as<double>();

    robot->getSimpleSetup().setStartAndGoalStates(
    	robot->getFullStateFromGeometricComponent(start),
    	robot->getFullStateFromGeometricComponent(goal), config["general/distanceToGoal"].as<double>());

    robot->getSimpleSetup().getSpaceInformation().get()->setPropagationStepSize(config["general/propStepSize"].as<double>());
    robot->getSimpleSetup().getSpaceInformation().get()->setMinMaxControlDuration(config["general/minControlDuration"].as<double>(),config["general/maxControlDuration"].as<double>());

    //Siempre después de los sets del step size y el min/max control duration, para que la dinamica que usa el muestreador de control, sea con esos parámetros.
	/*control::DirectedControlSamplerAllocator pla = boost::bind(auvplanning::PlanificadorLocal::PlanificadorLocalAllocator, robot->getSimpleSetup().getSpaceInformation().get(), 15, config);
    robot->getSimpleSetup().getSpaceInformation().get()->setDirectedControlSamplerAllocator(pla);*/
    

    //control::DirectedControlSamplerAllocator pla = boost::bind(auvplanning::AUVDirectedControlSampler::AUVDirectedControlSamplerAllocator, robot->getSimpleSetup().getSpaceInformation().get(), 15, config);
    //robot->getSimpleSetup().getSpaceInformation().get()->setDirectedControlSamplerAllocator(pla);

    robot->getSimpleSetup().getSpaceInformation()->setStateValidityCheckingResolution(config["general/checkresolution"].as<double>());

    robot->getSimpleSetup().getStateSpace()->registerProjection("AUVProjection",auvplanning::allocGeometricStateProjector(robot->getSimpleSetup().getStateSpace(),
     										robot->getGeometricComponentStateSpace(), robot->getGeometricStateExtractor()));

    printf ("FIN setup\n");
}

base::PlannerPtr myESTConfiguredPlanner(oauv::AUVRobotPtr& robot, double range)
{
    control::EST *est = new control::EST(robot->getSimpleSetup().getSpaceInformation());
    est->setRange(range);
    est->setProjectionEvaluator("AUVProjection");
    return base::PlannerPtr(est);
}

base::PlannerPtr myKPIECE1ConfiguredPlanner(oauv::AUVRobotPtr& robot)
{
    control::KPIECE1 *kpiece1 = new control::KPIECE1(robot->getSimpleSetup().getSpaceInformation());
    kpiece1->setProjectionEvaluator("AUVProjection");
    return base::PlannerPtr(kpiece1);
}


void AUVRobotDemo(oauv::AUVRobotPtr& robot, YAML::Node config)
{

	std:string planner_str = config["plan/planner"].as<std::string>();

	if (planner_str.compare("DynamicRRT") == 0){
		robot->getSimpleSetup().setPlanner(base::PlannerPtr(new auvplanning::DynamicRRT(robot->getSimpleSetup().getSpaceInformation())));		
	}else if (planner_str.compare("RRT") == 0){
		robot->getSimpleSetup().setPlanner(base::PlannerPtr(new control::RRT(robot->getSimpleSetup().getSpaceInformation())));		
	}else if (planner_str.compare("EST") == 0){
		base::PlannerPtr planner = myESTConfiguredPlanner(robot, config["plan/ESTrange"].as<double>());
		robot->getSimpleSetup().setPlanner(planner);
	}else if (planner_str.compare("KPIECE1") == 0){
		base::PlannerPtr planner = myKPIECE1ConfiguredPlanner(robot);
		robot->getSimpleSetup().setPlanner(planner);
	}else if (planner_str.compare("SyclopRRT") == 0){
		//robot->getSimpleSetup().setPlanner(base::PlannerPtr(new control::SyclopRRT(robot->getSimpleSetup().getSpaceInformation())));
	}else if (planner_str.compare("") == 0){
		
	}

	robot->setup(DCS_TYPE);

	robot->getSimpleSetup().getPlanner()->printProperties(std::cout);
	robot->getSimpleSetup().getPlanner()->printSettings(std::cout);
	robot->getSimpleSetup().getStateSpace()->printSettings(std::cout);

	std::fstream benchmarkFile;
	std::ofstream benchmarkWithQuatFile;
	std::ofstream benchmarkControls;
  	benchmarkFile.open (config["plan/solutionFile"].as<std::string>(), std::fstream::in | std::fstream::out | std::fstream::trunc);

	// try to solve the problem
	if (robot->getSimpleSetup().solve(config["plan/time"].as<double>()))
	{

		control::PathControl& path(robot->getSimpleSetup().getSolutionPath());
		path.printAsMatrix(std::cout);
		path.interpolate(); // uncomment if you want to plot the path
		path.printAsMatrix(benchmarkFile);
		if (!robot->getSimpleSetup().haveExactSolutionPath())
		{
			std::cout << "Solution is approximate. Distance to actual goal is " <<
					robot->getSimpleSetup().getProblemDefinition()->getSolutionDifference() << std::endl;
		}else{
			std::cout << "Solution is in the range. Distance to actual goal is " <<
					robot->getSimpleSetup().getProblemDefinition()->getSolutionDifference() << std::endl;
		}
	}

	/*unsigned int llamadas_colisionador = robot->getStateValidityChecker().get()->FCLStateValidityChecker::getCallCounter();

	printf("Llamadas al colisionador: %d\n", llamadas_colisionador);*/

  	benchmarkWithQuatFile.open (config["plan/solutionQuatFile"].as<std::string>(), std::fstream::out | std::fstream::trunc);
  	benchmarkControls.open (config["plan/solutionControlsFile"].as<std::string>(), std::fstream::out | std::fstream::trunc);

    benchmarkFile.seekg (0, ios::beg);
    double num = 0.0;
    int index = 0;
	while(benchmarkFile >> num){
		index++;

		switch(index){
			case 1:
			case 2:
			case 3:
			{
				benchmarkWithQuatFile << num;
				benchmarkWithQuatFile << " ";
				break;
			}
			case 4:	
			{
				fcl::Quaternion3f quat;
    			fcl::Vec3f zaxis(0., 0., 1.); //se pone así porque tiene que ser un vector unitario
    			quat.fromAxisAngle(zaxis, num);
				benchmarkWithQuatFile << quat.getX ();
				benchmarkWithQuatFile << " ";
				benchmarkWithQuatFile << quat.getY ();
				benchmarkWithQuatFile << " ";
				benchmarkWithQuatFile << quat.getZ ();
				benchmarkWithQuatFile << " ";
				benchmarkWithQuatFile << quat.getW ();
				benchmarkWithQuatFile << " ";
				break;
			}
			case 9:
			case 10:
			case 11:
			{
				benchmarkControls << num;
				benchmarkControls << " ";
				break;
			}
			case 12:
			{
				benchmarkControls << num;
				benchmarkControls << " ";
				index = 0;
				benchmarkWithQuatFile << "\n";
				benchmarkControls << "\n";
				break;
			}
		}		
	}

  	benchmarkFile.close();
  	benchmarkWithQuatFile.close();
  	benchmarkControls.close();
}

void AUVRobotBenchmark(oauv::AUVRobotPtr& robot, YAML::Node config)
{

	double runtime = config["benchmark/runtime"].as<double>();
	double runmemory = config["benchmark/runmemory"].as<double>();
	double runcount = config["benchmark/runcount"].as<double>();

	tools::Benchmark::Request request(runtime, runmemory, runcount); 
	request.displayProgress = config["benchmark/displayProgress"].as<bool>();
	request.useThreads = config["benchmark/useThreads"].as<bool>();

    robot->setup(DCS_TYPE);

    tools::Benchmark b(robot->getSimpleSetup(), config["benchmark/testname"].as<std::string>());
    //b.addExperimentParameter("save_paths", "INTEGER", "10")
    

    //std::vector<std::string> planners_v= config["benchmark/planners"];
    for(int it = 0; it < config["benchmark/planners"].size();it++) {

	   std::string value = config["benchmark/planners"][it].as<std::string>();

	   if (value.compare("RRT") == 0)
	   {
	   		b.addPlanner(base::PlannerPtr(new control::RRT(robot->getSimpleSetup().getSpaceInformation())));
	   		printf("Planificador RRT añadido al benchmark\n");
	   }else if (value.compare("EST") == 0)
	   {
	   		for(int it_est = 0; it_est < config["benchmark/ESTrange"].size();it_est++) {
	   			double value_est = config["benchmark/ESTrange"][it_est].as<double>();
	   			b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, robot ,value_est));
	   			printf("Planificador EST añadido al benchmark, range = %f\n",value_est);
	   		}
	   }else if (value.compare("KPIECE1") == 0)
	   {
			b.addPlannerAllocator(std::bind(&myKPIECE1ConfiguredPlanner, robot));	   		
	   		printf("Planificador KPIECE1 añadido al benchmark\n");
	   }else if (value.compare("PDST") == 0)
	   {
	   		b.addPlanner(base::PlannerPtr(new control::PDST(robot->getSimpleSetup().getSpaceInformation())));
	   		printf("Planificador PDST añadido al benchmark\n");
	   }else if (value.compare("SyclopRRT") == 0)
	   {
			//b.addPlanner(base::PlannerPtr(new control::SyclopRRT(robot->getSimpleSetup().getSpaceInformation())));
	   		
	   }else if (value.compare("SyclopEST") == 0)
	   {
			//b.addPlanner(base::PlannerPtr(new control::SyclopEST(robot->getSimpleSetup().getSpaceInformation())));
	   		
	   }
	   
	}

    b.benchmark(request);
    b.saveResultsToFile(config["benchmark/resultsFile"].as<std::string>().c_str());
}

void moveAUV(oauv::AUVRobotPtr& robot, YAML::Node config){

	robot->setup(DCS_TYPE);
	double t1 = config["dynamics/th1"].as<double>();
	double t2 = config["dynamics/th2"].as<double>();
	double t3 = config["dynamics/th3"].as<double>();
	double tiempo = config["dynamics/time"].as<double>();
	printf("[moveAUV] Init\n");

	base::State *state; 
	state = robot->getSimpleSetup().getSpaceInformation()->allocState();
	state->as<base::RealVectorStateSpace::StateType>()->values[0] = 0.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[2] = 0.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[3] = 0.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[4] = 0.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[5] = 0.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[6] = 0.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[7] = 0.0;
	printf("state x: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[0]);
	printf("state y: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[1]);
	printf("state z: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[2]);
	printf("state yaw: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[3]);
	printf("state vx: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[4]);
	printf("state vy: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[5]);
	printf("state vz: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[6]);
	printf("state vyaw: %f\n", state->as<base::RealVectorStateSpace::StateType>()->values[7]);

	std::vector<double> reals;
    robot->getSimpleSetup().getStateSpace()->copyToReals(reals, state); 
    printf("size of reals: %d\n", static_cast<int>(reals.size())); 


    control::Control *control_ = robot->getSimpleSetup().getSpaceInformation()->allocControl();
    if(!robot->getSimpleSetup().getSpaceInformation()->getControlSpace()->isCompound()){

	    control_->as<control::RealVectorControlSpace::ControlType>()->values[0] = t1;
	    control_->as<control::RealVectorControlSpace::ControlType>()->values[1] = t2;
	    control_->as<control::RealVectorControlSpace::ControlType>()->values[2] = t3;
		printf("control_ t1: %f\n", control_->as<control::RealVectorControlSpace::ControlType>()->values[0]);
		printf("control_ t2: %f\n", control_->as<control::RealVectorControlSpace::ControlType>()->values[1]);
		printf("control_ t3: %f\n", control_->as<control::RealVectorControlSpace::ControlType>()->values[2]);
    }else{	    

	    control_->as<control::CompoundControlSpace::ControlType>()->components[0]->as<control::RealVectorControlSpace::ControlType>()->values[0] = t1;
	    control_->as<control::CompoundControlSpace::ControlType>()->components[0]->as<control::RealVectorControlSpace::ControlType>()->values[1] = t2;
	    control_->as<control::CompoundControlSpace::ControlType>()->components[0]->as<control::RealVectorControlSpace::ControlType>()->values[2] = t3;
	    control_->as<control::CompoundControlSpace::ControlType>()->components[0]->as<control::RealVectorControlSpace::ControlType>()->values[3] = tiempo;
	    control_->as<control::CompoundControlSpace::ControlType>()->components[1]->as<control::RealVectorControlSpace::ControlType>()->values[0] = t1;
	    control_->as<control::CompoundControlSpace::ControlType>()->components[1]->as<control::RealVectorControlSpace::ControlType>()->values[1] = t2;
	    control_->as<control::CompoundControlSpace::ControlType>()->components[1]->as<control::RealVectorControlSpace::ControlType>()->values[2] = t3;
	    control_->as<control::CompoundControlSpace::ControlType>()->components[1]->as<control::RealVectorControlSpace::ControlType>()->values[3] = tiempo;
		/*printf("control_1 t1: 		%f\n", control_->getSubspace(0)->as<control::RealVectorControlSpace::ControlType>()->values[0]);
		printf("control_1 t2: 		%f\n", control_->getSubspace(0)->as<control::RealVectorControlSpace::ControlType>()->values[1]);
		printf("control_1 t3: 		%f\n", control_->getSubspace(0)->as<control::RealVectorControlSpace::ControlType>()->values[2]);
		printf("control_1 tiempo: 	%f\n", control_->getSubspace(0)->as<control::RealVectorControlSpace::ControlType>()->values[3]);
		printf("control_2 t1: 		%f\n", control_->getSubspace(1)->as<control::RealVectorControlSpace::ControlType>()->values[0]);
		printf("control_2 t2: 		%f\n", control_->getSubspace(1)->as<control::RealVectorControlSpace::ControlType>()->values[1]);
		printf("control_2 t3: 		%f\n", control_->getSubspace(1)->as<control::RealVectorControlSpace::ControlType>()->values[2]);
		printf("control_2 tiempo: 	%f\n", control_->getSubspace(1)->as<control::RealVectorControlSpace::ControlType>()->values[3]);*/
    }

    

    base::State *result = robot->getSimpleSetup().getSpaceInformation()->allocState();

    control::StatePropagatorPtr stProp = robot->getSimpleSetup().getSpaceInformation()->getStatePropagator();

	printf("[moveAUV] Start of propagate\n");
    stProp->propagate(state,control_,tiempo,result);
	printf("result x: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[0]);
	printf("result y: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[1]);
	printf("result z: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[2]);
	printf("result yaw: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[3]);
	printf("result vx: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[4]);
	printf("result vy: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[5]);
	printf("result vz: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[6]);
	printf("result vyaw: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[7]);
	printf("[moveAUV] End of propagate\n");
	printf("[moveAUV] -------------------\n");

	/*printf("[moveAUV] Start of sampleTo\n");

	stProp->canSteer() ? printf("Con steer\n") : printf("Sin steer\n");

	control::DirectedControlSamplerPtr controlSampler_ = robot->getSimpleSetup().getSpaceInformation()->allocDirectedControlSampler();

	//controlSampler_->as<SimpleDirectedControlSampler>()->setNumControlSamples(20);

	control::Control *rctrl = robot->getSimpleSetup().getSpaceInformation()->allocControl();
	unsigned int cd = controlSampler_->sampleTo(rctrl, state, result);
	printf("rctrl t1: %f\n", rctrl->as<control::RealVectorControlSpace::ControlType>()->values[0]);
	printf("rctrl t2: %f\n", rctrl->as<control::RealVectorControlSpace::ControlType>()->values[1]);
	printf("rctrl t3: %f\n", rctrl->as<control::RealVectorControlSpace::ControlType>()->values[2]);
	printf("result x: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[0]);
	printf("result y: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[1]);
	printf("result z: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[2]);
	printf("result yaw: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[3]);
	printf("result vx: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[4]);
	printf("result vy: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[5]);
	printf("result vz: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[6]);
	printf("result vyaw: %f\n", result->as<base::RealVectorStateSpace::StateType>()->values[7]);
	printf("rctrl tiempo: %d\n", cd);

	printf("[moveAUV] End of sampleTo\n");*/
}

int main(int argc, char** argv)
{


	if (argc < 2){
		printf("Por favor, introduzca una de las opciones para ejecutar el programa.\n");
		return 0;
	}else if(argc < 3){
		printf("Por favor, introduzca el fichero de configuracion.\n");
		return 0;
	}else{

		YAML::Node config = YAML::LoadFile(argv[2]);

		std::string env_fname 	=   config["general/env_file"].as<std::string>();
		std::string robot_fname = 	config["general/robot_file"].as<std::string>();

		oauv::AUVRobotPtr robot(new oauv::AUVRobot(false,config));

		if(strcmp(argv[1],"-d") == 0){
			AUVRobotSetup(robot,config);
			moveAUV(robot, config);	

		}else if(strcmp(argv[1],"-p") == 0){

			if(argc < 3){
				printf("Por favor, introduzca el archivo de configuración a utilizar.\n");
				return 0;
			}

			const char* configFileName = argv[2];

		    robot->getRigidBodyGeometry().setEnvironmentMesh(env_fname.c_str());
			robot->getRigidBodyGeometry().setRobotMesh(robot_fname.c_str());
			AUVRobotSetup(robot,config);
			AUVRobotDemo(robot,config);

		}else if(strcmp(argv[1],"-b") == 0){

			if(argc < 3){
				printf("Por favor, introduzca el archivo de configuración a utilizar.\n");				
				return 0;
			}

			const char* configFileName = argv[2];
			
		    robot->getRigidBodyGeometry().setEnvironmentMesh(env_fname.c_str());
			robot->getRigidBodyGeometry().setRobotMesh(robot_fname.c_str());
			AUVRobotSetup(robot,config);
			AUVRobotBenchmark(robot,config);

		}else if(strcmp(argv[1],"-s") == 0){

			if(argc < 3){
				printf("Por favor, introduzca las tres actuaciones para ejecutar la dinámica.\n");
				return 0;
			}

		    robot->getRigidBodyGeometry().setEnvironmentMesh(env_fname.c_str());
			robot->getRigidBodyGeometry().setRobotMesh(robot_fname.c_str());
			AUVRobotSetup(robot,config);

		}else{
			printf("Opción desconocida.\n");
			return 0;
		}
	}

	return 0;

}
