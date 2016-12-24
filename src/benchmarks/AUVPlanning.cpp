#include "benchmarks/AUVPlanning.h"

using namespace ompl;
namespace oauv = ompl::auvplanning;

int Main::runCounter = 0;
YAML::Node Main::config;

void Main::AUVRobotSetup(){
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

base::PlannerPtr myESTConfiguredPlanner(ompl::auvplanning::AUVRobotPtr robot, double range){
    control::EST *est = new control::EST(robot->getSimpleSetup().getSpaceInformation());
    std::string name = "EST" + std::to_string((int)range);
    est->setName(name);
    est->setRange(range);
    est->setProjectionEvaluator("AUVProjection");
    return base::PlannerPtr(est);
}

base::PlannerPtr myKPIECE1ConfiguredPlanner(ompl::auvplanning::AUVRobotPtr robot){
    control::KPIECE1 *kpiece1 = new control::KPIECE1(robot->getSimpleSetup().getSpaceInformation());
    kpiece1->setProjectionEvaluator("AUVProjection");
    return base::PlannerPtr(kpiece1);
}

base::PlannerPtr myPDSTConfiguredPlanner(ompl::auvplanning::AUVRobotPtr robot){
    control::PDST *pdst = new control::PDST(robot->getSimpleSetup().getSpaceInformation());
    pdst->setProjectionEvaluator("AUVProjection");
    return base::PlannerPtr(pdst);
}

/*
double distanceByPropagation(const base::State initialState, const base::State goalState){
	double result = 10000;
	//if()
	return result;
}*/

void Main::AUVRobotDemo(){

	std::string planner_str = config["plan/planner"].as<std::string>();
	std::string solutionFile_str = config["plan/solutionFile"].as<std::string>();
	std::string solutionQuatFile_str = config["plan/solutionQuatFile"].as<std::string>();
	std::string solutionControlsFile_str = config["plan/solutionControlsFile"].as<std::string>();

	bool hasController = false;

	if (planner_str.compare("DynamicRRT") == 0){
		robot->getSimpleSetup().setPlanner(base::PlannerPtr(new auvplanning::DynamicRRT(robot->getSimpleSetup().getSpaceInformation())));
		hasController = true;		
	}else if (planner_str.compare("RRT") == 0){
		robot->getSimpleSetup().setPlanner(base::PlannerPtr(new control::RRT(robot->getSimpleSetup().getSpaceInformation())));		
	}else if (planner_str.compare("EST") == 0){
		base::PlannerPtr planner = myESTConfiguredPlanner(getRobot(),config["plan/ESTrange"].as<double>());
		robot->getSimpleSetup().setPlanner(planner);
	}else if (planner_str.compare("KPIECE1") == 0){
		base::PlannerPtr planner = myKPIECE1ConfiguredPlanner(getRobot());
		robot->getSimpleSetup().setPlanner(planner);
	}else if (planner_str.compare("PDST") == 0){
		base::PlannerPtr planner = myPDSTConfiguredPlanner(getRobot());
		robot->getSimpleSetup().setPlanner(planner);
	}else if (planner_str.compare("SyclopRRT") == 0){
		//robot->getSimpleSetup().setPlanner(base::PlannerPtr(new control::SyclopRRT(robot->getSimpleSetup().getSpaceInformation())));
	}else if (planner_str.compare("") == 0){
		
	}

	robot->setup(getTypeControlSampler(config["plan/controlSampler"].as<std::string>()));

	robot->getSimpleSetup().getPlanner()->printProperties(std::cout);
	robot->getSimpleSetup().getPlanner()->printSettings(std::cout);
	robot->getSimpleSetup().getStateSpace()->printSettings(std::cout);

	std::fstream benchmarkFile;
  	benchmarkFile.open (solutionFile_str, std::fstream::in | std::fstream::out | std::fstream::trunc);

	// try to solve the problem
	if (robot->getSimpleSetup().solve(config["plan/time"].as<double>()))
	{

		if(!hasController){
			control::PathControl& path(robot->getSimpleSetup().getSolutionPath());
			path.printAsMatrix(std::cout);
			path.interpolate(); // uncomment if you want to plot the path
			std::cout << "PathControl Interpolación hecha" << std::endl;
			path.printAsMatrix(benchmarkFile);
			//benchmarkFile.close();
    		printSolutionToFile(benchmarkFile, solutionQuatFile_str, solutionControlsFile_str, false);
		}else{
			const base::PathPtr &p = robot->getSimpleSetup().getProblemDefinition()->getSolutionPath();
			auvplanning::PathController& pathC(static_cast<auvplanning::PathController&>(*p));
			//pathC.printAsMatrix(std::cout);
			//std::cout << "PathController Impresión hecha" << std::endl;
			pathC.interpolate(); // uncomment if you want to plot the path
			std::cout << "PathController Interpolación hecha" << std::endl;
			//pathC.printAsMatrix(std::cout);
			pathC.printAsMatrix(benchmarkFile);
			std::cout << "PathController Impresión hecha" << std::endl;
			//benchmarkFile.close();
    		printSolutionToFile(benchmarkFile, solutionQuatFile_str, solutionControlsFile_str, true);
		}
		benchmarkFile.close();

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
  	
}

// Assume these functions are defined
void optionalPreRunEvent(const base::PlannerPtr &planner)
{
    // do whatever configuration we want to the planner,
    // including changing of problem definition (input states)
    // via planner->getProblemDefinition()
}
void optionalPostRunEvent(const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run)
{
	YAML::Node config = Main::getConfig();
	int run_iterator = Main::getRunCounter();

	std::string folder_str = "./paths_" + config["benchmark/solutionFile"].as<std::string>();
	chdir(folder_str.c_str());

    std::string solutionFile_str = config["benchmark/solutionFile"].as<std::string>() + "_" + std::to_string(run_iterator) + ".txt";
	std::string solutionQuatFile_str = config["benchmark/solutionQuatFile"].as<std::string>() + "_" + std::to_string(run_iterator) + ".txt";
	std::string solutionControlsFile_str = config["benchmark/solutionControlsFile"].as<std::string>() + "_" + std::to_string(run_iterator) + ".txt";

	std::fstream benchmarkFile;
  	benchmarkFile.open (solutionFile_str, std::fstream::in | std::fstream::out | std::fstream::trunc);
    
    const base::PathPtr &p = planner->getProblemDefinition()->getSolutionPath();
    if(p){
    	control::PathControl& path(static_cast<control::PathControl&>(*p));
		//path.printAsMatrix(std::cout);
		path.interpolate(); // uncomment if you want to plot the path
		//OMPL_INFORM("Solucion " + solutionFile_str.c_str() + " guardada");
		path.printAsMatrix(benchmarkFile);
	    Main::printSolutionToFile(benchmarkFile, solutionQuatFile_str, solutionControlsFile_str, false);
	    benchmarkFile.close();
	    chdir("..");
	}
    Main::addRun();
}

void Main::AUVRobotBenchmark(){

	std::string folder_str = "./paths_" + config["benchmark/solutionFile"].as<std::string>();
	mkdir(folder_str.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

	double runtime = config["benchmark/runtime"].as<double>();
	double runmemory = config["benchmark/runmemory"].as<double>();
	double runcount = config["benchmark/runcount"].as<double>();

	tools::Benchmark::Request request(runtime, runmemory, runcount); 
	request.displayProgress = config["benchmark/displayProgress"].as<bool>();
	request.useThreads = config["benchmark/useThreads"].as<bool>();

    robot->setup(getTypeControlSampler(config["benchmark/controlSampler"].as<std::string>()));

    tools::Benchmark b(robot->getSimpleSetup(), config["benchmark/testname"].as<std::string>());
    //b.addExperimentParameter("save_paths", "INTEGER", "10")
    b.addExperimentParameter("num_dofs", "INTEGER", "8");
    b.addExperimentParameter("distance_to_goal_to_be_considered_achieved", "REAL", config["general/distanceToGoal"].as<std::string>());
    b.addExperimentParameter("propagation_step_size", "INTEGER", config["general/propStepSize"].as<std::string>());
    b.addExperimentParameter("minimum_control_duration", "INTEGER", config["general/minControlDuration"].as<std::string>());
    b.addExperimentParameter("maximum_control_duration", "REAL", config["general/maxControlDuration"].as<std::string>());
    b.addExperimentParameter("collision_checking_resolution", "REAL", config["general/checkresolution"].as<std::string>());
	b.addExperimentParameter("nombre_del_archivo_del_entorno", "STRING", config["general/env_file"].as<std::string>());
	b.addExperimentParameter("nombre_del_archivo_del_robot", "STRING", config["general/robot_file"].as<std::string>());

    //std::vector<std::string> planners_v= config["benchmark/planners"];
    for(int it = 0; it < config["benchmark/planners"].size();it++) {

	   std::string value = config["benchmark/planners"][it].as<std::string>();

	   if (value.compare("DynamicRRT") == 0){
			b.addPlanner(base::PlannerPtr(new auvplanning::DynamicRRT(robot->getSimpleSetup().getSpaceInformation())));		
	   }if (value.compare("RRT") == 0)
	   {
	   		b.addPlanner(base::PlannerPtr(new control::RRT(robot->getSimpleSetup().getSpaceInformation())));
	   		printf("Planificador RRT añadido al benchmark\n");
	   }else if (value.compare("EST") == 0)
	   {
	   		for(int it_est = 0; it_est < config["benchmark/ESTrange"].size();it_est++) {
	   			double value_est = config["benchmark/ESTrange"][it_est].as<double>();
	   			b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, getRobot(), value_est));
	   			printf("Planificador EST añadido al benchmark, range = %f\n",value_est);
	   			b.addExperimentParameter("EST range " + it_est, "INTEGER", config["benchmark/ESTrange"][it_est].as<std::string>());
	   		}
	   }else if (value.compare("KPIECE1") == 0)
	   {
			b.addPlannerAllocator(std::bind(&myKPIECE1ConfiguredPlanner, getRobot()));	   		
	   		printf("Planificador KPIECE1 añadido al benchmark\n");
	   }else if (value.compare("PDST") == 0)
	   {	   	
			b.addPlannerAllocator(std::bind(&myPDSTConfiguredPlanner, getRobot()));	   		
	   		printf("Planificador PDST añadido al benchmark\n");
	   }else if (value.compare("SyclopRRT") == 0)
	   {
			//b.addPlanner(base::PlannerPtr(new control::SyclopRRT(robot->getSimpleSetup().getSpaceInformation())));
	   		
	   }else if (value.compare("SyclopEST") == 0)
	   {
			//b.addPlanner(base::PlannerPtr(new control::SyclopEST(robot->getSimpleSetup().getSpaceInformation())));
	   		
	   }
	   
	}


	// After the Benchmark class is defined, the events can be optionally registered:
	//b.setPreRunEvent(std::bind(&optionalPreRunEvent, std::placeholders::_1));
	b.setPostRunEvent(std::bind(&optionalPostRunEvent, std::placeholders::_1, std::placeholders::_2));

    b.benchmark(request);
    chdir(folder_str.c_str());
    b.saveResultsToFile(config["benchmark/resultsFile"].as<std::string>().c_str());
    chdir("..");
}

void Main::moveAUV(){

	robot->setup(getTypeControlSampler(config["plan/controlSampler"].as<std::string>()));
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

void Main::controlAUV(){

	robot->setup(getTypeControlSampler(config["plan/controlSampler"].as<std::string>()));
	printf("[controlAUV] Init\n");

	base::State *state; 
	state = robot->getSimpleSetup().getSpaceInformation()->allocState();
	state->as<base::RealVectorStateSpace::StateType>()->values[0] = 600.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[1] = 120.0;
	state->as<base::RealVectorStateSpace::StateType>()->values[2] = 100.0;
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

	base::State *reference; 
	reference = robot->getSimpleSetup().getSpaceInformation()->allocState();
	reference->as<base::RealVectorStateSpace::StateType>()->values[0] = 550.0;
	reference->as<base::RealVectorStateSpace::StateType>()->values[1] = 330.0;
	reference->as<base::RealVectorStateSpace::StateType>()->values[2] = 100.0;
	reference->as<base::RealVectorStateSpace::StateType>()->values[3] = 0.0;
	reference->as<base::RealVectorStateSpace::StateType>()->values[4] = 0.0;
	reference->as<base::RealVectorStateSpace::StateType>()->values[5] = 0.0;
	reference->as<base::RealVectorStateSpace::StateType>()->values[6] = 0.0;
	reference->as<base::RealVectorStateSpace::StateType>()->values[7] = 0.0;
	printf("reference x: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[0]);
	printf("reference y: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[1]);
	printf("reference z: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[2]);
	printf("reference yaw: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[3]);
	printf("reference vx: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[4]);
	printf("reference vy: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[5]);
	printf("reference vz: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[6]);
	printf("reference vyaw: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[7]);

    controller::ControllerPtr controller(new controller::AUV2StepPID(robot->getSimpleSetup().getSpaceInformation().get()));

    const unsigned int maxDuration = robot->getSimpleSetup().getSpaceInformation()->getMaxControlDuration();

	printf("[controlAUV] Start of controller\n");
    controller->propagateController(state, reference, maxDuration);
	printf("result x: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[0]);
	printf("result y: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[1]);
	printf("result z: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[2]);
	printf("result yaw: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[3]);
	printf("result vx: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[4]);
	printf("result vy: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[5]);
	printf("result vz: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[6]);
	printf("result vyaw: %f\n", reference->as<base::RealVectorStateSpace::StateType>()->values[7]);
	printf("[controlAUV] End of controller\n");
	printf("[controlAUV] -------------------\n");

	robot->getSimpleSetup().getSpaceInformation()->freeState(state);
	robot->getSimpleSetup().getSpaceInformation()->freeState(reference);
}

void Main::checkState(){
	robot->setup(getTypeControlSampler(config["plan/controlSampler"].as<std::string>()));
	printf("[checkState] Init\n");

	base::State *state; 
	state = robot->getSimpleSetup().getSpaceInformation()->allocState();
	state->as<base::RealVectorStateSpace::StateType>()->values[0] = config["general/start"][0].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[1] = config["general/start"][1].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[2] = config["general/start"][2].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[3] = config["general/start"][3].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[4] = config["general/start"][4].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[5] = config["general/start"][5].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[6] = config["general/start"][6].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[7] = config["general/start"][7].as<double>();
	robot->getSimpleSetup().getSpaceInformation()->printState(state);
	bool isValid = robot->getStateValidityChecker()->isValid(state);
	isValid? printf("Es válido\n"): printf("No es válido\n");;
}

int Main::getTypeControlSampler(std::string string_value){
	int value = -1;
    value = (string_value.compare("Random") == 0)? AUV_SIMPLE_DCS : -1;
    value = (string_value.compare("SemiRandom") == 0)? AUV_SEMI_RANDOM_DCS : -1;
    //value = (string_value.compare("AUV_DIRECTED_DCS") == 0)? AUV_DIRECTED_DCS : -1;
    value = (string_value.compare("PID") == 0)? AUV_PID_DCS : -1;
    value = (string_value.compare("2StepsPID") == 0)? AUV_2PID_DCS : -1;

    return value;	
}

void Main::printSolutionToFile(std::fstream &solutionFile, std::string quatFile_str, std::string controlsFile_str, bool modeController){
	std::ofstream quatFile;
	std::ofstream controlsFile;

  	quatFile.open (quatFile_str, std::fstream::out | std::fstream::trunc);
  	controlsFile.open (controlsFile_str, std::fstream::out | std::fstream::trunc);

	solutionFile.seekg (0, ios::beg);
    double num = 0.0;
    int index = 0;
	while(solutionFile >> num){
		index++;
		if(!modeController){
			switch(index){
				case 1:
				case 2:
				case 3:
				{
					quatFile << num;
					quatFile << " ";
					break;
				}
				case 4:	
				{
					fcl::Quaternion3f quat;
	    			fcl::Vec3f zaxis(0., 0., 1.); //se pone así porque tiene que ser un vector unitario
	    			quat.fromAxisAngle(zaxis, num);
					quatFile << quat.getX ();
					quatFile << " ";
					quatFile << quat.getY ();
					quatFile << " ";
					quatFile << quat.getZ ();
					quatFile << " ";
					quatFile << quat.getW ();
					quatFile << " ";
					break;
				}
				case 9:
				case 10:
				case 11:
				{
					controlsFile << num;
					controlsFile << " ";
					break;
				}
				case 12:
				{
					controlsFile << num;
					controlsFile << " ";
					index = 0;
					quatFile << "\n";
					controlsFile << "\n";
					break;
				}
			}		
		}else{
			switch(index){
				case 1:
				case 2:
				case 3:
				{
					quatFile << num;
					quatFile << " ";
					break;
				}
				case 4:	
				{
					fcl::Quaternion3f quat;
	    			fcl::Vec3f zaxis(0., 0., 1.); //se pone así porque tiene que ser un vector unitario
	    			quat.fromAxisAngle(zaxis, num);
					quatFile << quat.getX ();
					quatFile << " ";
					quatFile << quat.getY ();
					quatFile << " ";
					quatFile << quat.getZ ();
					quatFile << " ";
					quatFile << quat.getW ();
					quatFile << " ";
					break;
				}
				case 9:
				case 10:
				case 11:
				{
					controlsFile << num;
					controlsFile << " ";
					break;
				}
				case 12:
				{
					fcl::Quaternion3f quat;
	    			fcl::Vec3f zaxis(0., 0., 1.); //se pone así porque tiene que ser un vector unitario
	    			quat.fromAxisAngle(zaxis, num);
					controlsFile << quat.getX ();
					controlsFile << " ";
					controlsFile << quat.getY ();
					controlsFile << " ";
					controlsFile << quat.getZ ();
					controlsFile << " ";
					controlsFile << quat.getW ();
					controlsFile << " ";
					break;
				}
				case 17:
				{					
					quatFile << "\n";
					controlsFile << num << "\n";
					index = 0;
					break;
				}
			}
		}
	}

  	quatFile.close();
  	controlsFile.close();
}

Main::Main(YAML::Node configuration):
robot(new oauv::AUVRobot(false,configuration))/*, config(configuration)*/{
	/*robot = new oauv::AUVRobotPtr robot(new oauv::AUVRobot(false,configuration));*/
	config = configuration;
}

int main(int argc, char** argv){
	if (argc < 2){
		printf("Por favor, introduzca una de las opciones para ejecutar el programa.\n");
	}else if(argc < 3){

		if(strcmp(argv[1],"-h") == 0){
			printf("Opciones disponibles:\n");
			printf("-d : \n");
			printf("-c : \n");
			printf("-p : \n");
			printf("-b : \n");
			printf("-v : \n");
			printf("-h : \n");
		}else{
			printf("Por favor, introduzca el fichero de configuracion.\n");
		}
	}else{

		YAML::Node config = YAML::LoadFile(argv[2]);

		std::string env_fname 	= config["general/env_file"].as<std::string>();
		std::string robot_fname = config["general/robot_file"].as<std::string>();

		//oauv::AUVRobotPtr robot(new oauv::AUVRobot(false,config));
		Main test(config);

		if(strcmp(argv[1],"-d") == 0){
			test.AUVRobotSetup();
			test.moveAUV();	

		}else if(strcmp(argv[1],"-c") == 0){
			test.AUVRobotSetup();
			test.controlAUV();	

		}else if(strcmp(argv[1],"-p") == 0){
		    test.getRobot()->getRigidBodyGeometry().setEnvironmentMesh(env_fname.c_str());
			test.getRobot()->getRigidBodyGeometry().setRobotMesh(robot_fname.c_str());
			test.AUVRobotSetup();
			test.AUVRobotDemo();

		}else if(strcmp(argv[1],"-b") == 0){
		    test.getRobot()->getRigidBodyGeometry().setEnvironmentMesh(env_fname.c_str());
			test.getRobot()->getRigidBodyGeometry().setRobotMesh(robot_fname.c_str());
			test.AUVRobotSetup();
			test.AUVRobotBenchmark();

		}else if(strcmp(argv[1],"-v") == 0){
		    test.getRobot()->getRigidBodyGeometry().setEnvironmentMesh(env_fname.c_str());
			test.getRobot()->getRigidBodyGeometry().setRobotMesh(robot_fname.c_str());
			test.AUVRobotSetup();
			test.checkState();

		}else{
			printf("Opción desconocida.\n");
		}
	}

	return 0;
}
