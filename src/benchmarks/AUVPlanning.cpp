#include "benchmarks/AUVPlanning.h"

using namespace ompl;
namespace oauv = ompl::auvplanning;

int 								Main::runCounter = 0;
YAML::Node 							Main::config;
ompl::auvplanning::AUVRobotPtr 		Main::robot;

std::string getStringTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  std::time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%Y_%m_%d_%H:%M:%S",timeinfo);
  std::string str(buffer);

  return str;
}

void Main::AUVRobotSetup()
{
	base::StateSpacePtr StSpace(robot->getSimpleSetup().getStateSpace());

	// define start state
	base::ScopedState<base::RealVectorStateSpace> start(StSpace);
	start[0] = config["general/start"][0].as<double>();
	start[1] = config["general/start"][1].as<double>();
	start[2] = config["general/start"][2].as<double>();	
	start[3] = config["general/start"][3].as<double>();	
	start[4] = config["general/start"][4].as<double>();	
	start[5] = config["general/start"][5].as<double>();	
	start[6] = config["general/start"][6].as<double>();	
	start[7] = config["general/start"][7].as<double>();	

	// define goal state
	base::ScopedState<base::RealVectorStateSpace> goalState(StSpace);
    goalState[0] = config["general/goal"][0].as<double>();
    goalState[1] = config["general/goal"][1].as<double>();
    goalState[2] = config["general/goal"][2].as<double>();
    goalState[3] = config["general/goal"][3].as<double>();
    goalState[4] = config["general/goal"][4].as<double>();
    goalState[5] = config["general/goal"][5].as<double>();
    goalState[6] = config["general/goal"][6].as<double>();
    goalState[7] = config["general/goal"][7].as<double>();

    base::GoalPtr goal(new oauv::AUVGoalState(robot->getSimpleSetup().getSpaceInformation()));
    static_cast<base::GoalState*>(goal.get())->setState(goalState);
    static_cast<base::GoalState*>(goal.get())->setThreshold(config["general/distanceToGoal"].as<double>());

    robot->getSimpleSetup().addStartState(start);
    robot->getSimpleSetup().setGoal(goal);
    double gTh = static_cast<base::GoalState*>(robot->getSimpleSetup().getGoal().get())->getThreshold();
    OMPL_INFORM("Threshold: %f", gTh);

    robot->getSimpleSetup().getSpaceInformation().get()->setPropagationStepSize(config["general/propagationStepSize"].as<double>());
    double stpSize = robot->getSimpleSetup().getSpaceInformation().get()->getPropagationStepSize();
    OMPL_INFORM("Propagation Step size: %f", stpSize);
    robot->getSimpleSetup().getSpaceInformation().get()->setMinMaxControlDuration(config["general/minControlDuration"].as<double>(),config["general/maxControlDuration"].as<double>());
  
    robot->getSimpleSetup().getSpaceInformation()->setStateValidityCheckingResolution(config["general/checkResolution"].as<double>());

    base::ProjectionEvaluatorPtr auvProjection = auvplanning::allocGeometricStateProjector(robot->getSimpleSetup().getStateSpace(), robot->getGeometricComponentStateSpace(),robot->getGeometricStateExtractor());
    robot->getSimpleSetup().getStateSpace()->registerProjection("AUVProjection", auvProjection);

}

base::PlannerPtr myESTConfiguredPlanner(ompl::auvplanning::AUVRobotPtr robot, double range)
{
    control::EST *est = new control::EST(robot->getSimpleSetup().getSpaceInformation());
    std::string name = "EST_" + std::to_string((int)range);
    est->setName(name);
    est->setRange(range);
    est->setProjectionEvaluator("AUVProjection");
    return base::PlannerPtr(est);
}

base::PlannerPtr myKPIECE1ConfiguredPlanner(ompl::auvplanning::AUVRobotPtr robot, double borderFraction, double goodCellScoreFactor, double badCellScoreFactor, double maxCloseSampleCount)
{
    control::KPIECE1 *kpiece1 = new control::KPIECE1(robot->getSimpleSetup().getSpaceInformation());

	stringstream stream;
	stream << fixed << setprecision(2) << borderFraction << "_" << goodCellScoreFactor << "_" << badCellScoreFactor;
	string name_str = stream.str();
    std::string name = "KPIECE1_" + name_str + "_" + std::to_string((int)maxCloseSampleCount);
    kpiece1->setName(name);

    kpiece1->setProjectionEvaluator("AUVProjection");
    kpiece1->setBorderFraction(borderFraction);
    kpiece1->setCellScoreFactor(goodCellScoreFactor, badCellScoreFactor);
    kpiece1->setMaxCloseSamplesCount((int)maxCloseSampleCount);
    return base::PlannerPtr(kpiece1);
}

base::PlannerPtr myPDSTConfiguredPlanner(ompl::auvplanning::AUVRobotPtr robot)
{
    control::PDST *pdst = new control::PDST(robot->getSimpleSetup().getSpaceInformation());
    pdst->setProjectionEvaluator("AUVProjection");
    return base::PlannerPtr(pdst);
}

void Main::AUVRobotDemo()
{
	std::string planner_str = config["general/planners"][0].as<std::string>();
	std::string solutionFile_str = config["plan/solutionFile"].as<std::string>();
	std::string solutionQuatFile_str = config["plan/solutionQuatFile"].as<std::string>();
	std::string solutionControlsFile_str = config["plan/solutionControlsFile"].as<std::string>();

	bool hasController = false;

	if (planner_str.compare("DynamicRRT") == 0)
	{
		robot->getSimpleSetup().setPlanner(base::PlannerPtr(new auvplanning::DynamicRRT(robot->getSimpleSetup().getSpaceInformation())));
		hasController = true;		
	}else if (planner_str.compare("RRT") == 0)
	{
		robot->getSimpleSetup().setPlanner(base::PlannerPtr(new control::RRT(robot->getSimpleSetup().getSpaceInformation())));		
	}else if (planner_str.compare("EST") == 0)
	{
		base::PlannerPtr planner = myESTConfiguredPlanner(getRobot(),config["EST/Range"][0].as<double>());
		robot->getSimpleSetup().setPlanner(planner);
	}else if (planner_str.compare("KPIECE1") == 0)
	{
		double bf = config["KPIECE1/borderFraction"][0].as<double>();
		double goodCSF = config["KPIECE1/CellScoreFactor"][0].as<double>();
		double badCSF = config["KPIECE1/CellScoreFactor"][1].as<double>();
		int    mcsc= config["KPIECE1/MaxCloseSampleCount"][0].as<double>();
		base::PlannerPtr planner = myKPIECE1ConfiguredPlanner(getRobot(), bf, goodCSF, badCSF, mcsc);
		robot->getSimpleSetup().setPlanner(planner);
	}else if (planner_str.compare("PDST") == 0)
	{
		base::PlannerPtr planner = myPDSTConfiguredPlanner(getRobot());
		robot->getSimpleSetup().setPlanner(planner);
	}else if (planner_str.compare("SyclopRRT") == 0)
	{
		//robot->getSimpleSetup().setPlanner(base::PlannerPtr(new control::SyclopRRT(robot->getSimpleSetup().getSpaceInformation())));
	}else if (planner_str.compare("") == 0){
		
	}

	robot->setup(getTypeControlSampler(config["general/controlSampler"].as<std::string>()));

	base::ProjectionEvaluatorPtr auvProjection = robot->getSimpleSetup().getStateSpace()->getProjection("AUVProjection");
	base::RealVectorBounds bounds_ = static_cast<base::RealVectorStateSpace*>(robot->getGeometricComponentStateSpace().get())->getBounds();
    const std::vector<double> b = bounds_.getDifference();
    printf("Tamaño del geometric state space: %f, %f, %f\n", b[0], b[1], b[2]);
    double numCells = config["general/numberOfCellsInProjectionDimension"].as<double>();
    std::vector<double> cellSizes(3);
    cellSizes[0] = b[0] / numCells;
    cellSizes[1] = b[1] / numCells;
    cellSizes[2] = b[2] / numCells;
    printf("Tamaño de cellSizes: %f, %f, %f\n", cellSizes[0], cellSizes[1], cellSizes[2]);
    auvProjection->setCellSizes(cellSizes);


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
		}else
		{
			const base::PathPtr &p = robot->getSimpleSetup().getProblemDefinition()->getSolutionPath();
			auvplanning::PathController& pathC(static_cast<auvplanning::PathController&>(*p));
			pathC.printAsMatrix(std::cout);
			std::cout << "PathController Impresión hecha" << std::endl;
			/*double prevStepSize = robot->getSimpleSetup().getSpaceInformation()->getPropagationStepSize();
			robot->getSimpleSetup().getSpaceInformation()->setPropagationStepSize(0.01);*/
			pathC.interpolate(); // uncomment if you want to plot the path
			//robot->getSimpleSetup().getSpaceInformation()->setPropagationStepSize(prevStepSize);
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
		}else
		{
			std::cout << "Solution is in the range. Distance to actual goal is " <<
					robot->getSimpleSetup().getProblemDefinition()->getSolutionDifference() << std::endl;
		}
	}

	int numeroLlamadasColisionador = robot->getCollisionCounter();
	OMPL_INFORM("Numero de llamadas al colisionador:  %d", numeroLlamadasColisionador);
	robot->resetCollisionCounter();
	int numeroLlamadasColisionador2 = robot->getCollisionCounter();
	OMPL_INFORM("Numero de llamadas al colisionador despues de reset:  %d", numeroLlamadasColisionador2);
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

    int numeroLlamadasColisionador = Main::getRobot()->getCollisionCounter();
    run["numero de llamadas al colisionador INTEGER"] = std::to_string(numeroLlamadasColisionador);
	Main::getRobot()->resetCollisionCounter();

    std::string fecha = getStringTime();
    std::string nombre_planificador = planner->getName();
    std::string solutionFile_str = fecha + "_" + nombre_planificador + "_" + std::to_string(run_iterator) + "_EulerPath.txt";
	std::string solutionQuatFile_str = fecha + "_" + nombre_planificador + "_" + std::to_string(run_iterator) + "_QuatPath.txt";
	std::string solutionControlsFile_str = fecha + "_" + nombre_planificador + "_" + std::to_string(run_iterator) + "_Controls.txt";

	std::string folder_str = "./paths_" + config["benchmark/pathFolder"].as<std::string>();
	int res1 = chdir(folder_str.c_str());
	
    /*std::string solutionFile_str = config["benchmark/solutionFile"].as<std::string>() + "_" + std::to_string(run_iterator) + ".txt";
	std::string solutionQuatFile_str = config["benchmark/solutionQuatFile"].as<std::string>() + "_" + std::to_string(run_iterator) + ".txt";
	std::string solutionControlsFile_str = config["benchmark/solutionControlsFile"].as<std::string>() + "_" + std::to_string(run_iterator) + ".txt";*/

	std::fstream benchmarkFile;
  	benchmarkFile.open (solutionFile_str, std::fstream::in | std::fstream::out | std::fstream::trunc);
    
    const base::PathPtr &p = planner->getProblemDefinition()->getSolutionPath();
    if(p && planner->getName().compare("DynamicRRT") != 0)
    {    	
    	control::PathControl& path(static_cast<control::PathControl&>(*p));
		path.interpolate(); 
		path.printAsMatrix(benchmarkFile);
	    Main::printSolutionToFile(benchmarkFile, solutionQuatFile_str, solutionControlsFile_str, false);
	}
	else if(p && planner->getName().compare("DynamicRRT") == 0)
	{			
		auvplanning::PathController& pathC(static_cast<auvplanning::PathController&>(*p));
		//pathC.printAsMatrix(std::cout);
		pathC.interpolate();
		pathC.printAsMatrix(benchmarkFile);
		Main::printSolutionToFile(benchmarkFile, solutionQuatFile_str, solutionControlsFile_str, true);
	}
	benchmarkFile.close();
	int res2 = chdir("..");

    Main::addRun();
}

void Main::AUVRobotBenchmark()
{

	std::string folder_str = "./paths_" + config["benchmark/pathFolder"].as<std::string>();
	mkdir(folder_str.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

	double runtime = config["benchmark/runtime"].as<double>();
	double runmemory = config["benchmark/runmemory"].as<double>();
	double runcount = config["benchmark/runcount"].as<double>();

	tools::Benchmark::Request request(runtime, runmemory, runcount); 
	request.displayProgress = config["benchmark/displayProgress"].as<bool>();
	request.useThreads = config["benchmark/useThreads"].as<bool>();

    robot->setup(getTypeControlSampler(config["general/controlSampler"].as<std::string>()));

	base::ProjectionEvaluatorPtr auvProjection = robot->getSimpleSetup().getStateSpace()->getProjection("AUVProjection");
	base::RealVectorBounds bounds_ = static_cast<base::RealVectorStateSpace*>(robot->getGeometricComponentStateSpace().get())->getBounds();
    const std::vector<double> bounds = bounds_.getDifference();
    double numCells = config["general/numberOfCellsInProjectionDimension"].as<double>();
    std::vector<double> cellSizes(3);
    cellSizes[0] = bounds[0] / numCells;
    cellSizes[1] = bounds[1] / numCells;
    cellSizes[2] = bounds[2] / numCells;
    OMPL_INFORM("Projection cell sizes: %f, %f, %f\n", cellSizes[0], cellSizes[1], cellSizes[2]);
    auvProjection->setCellSizes(cellSizes);

    tools::Benchmark b(robot->getSimpleSetup(), config["benchmark/testname"].as<std::string>());
    //b.addExperimentParameter("save_paths", "INTEGER", "10")
    b.addExperimentParameter("num_dofs", "INTEGER", "8");
	b.addExperimentParameter("muestreador_de_control", "STRING", config["general/controlSampler"].as<std::string>());
	b.addExperimentParameter("funcion_de_distancia", "STRING", config["general/distanceFunction"].as<std::string>());
    b.addExperimentParameter("number of cells per dimention in the projection", "INTEGER", config["general/numberOfCellsInProjectionDimension"].as<std::string>());
    b.addExperimentParameter("distance_to_goal_to_be_considered_achieved", "REAL", config["general/distanceToGoal"].as<std::string>());
    b.addExperimentParameter("propagation_step_size", "INTEGER", config["general/propagationStepSize"].as<std::string>());
    b.addExperimentParameter("minimum_control_duration", "INTEGER", config["general/minControlDuration"].as<std::string>());
    b.addExperimentParameter("maximum_control_duration", "REAL", config["general/maxControlDuration"].as<std::string>());
    b.addExperimentParameter("collision_checking_resolution", "REAL", config["general/checkResolution"].as<std::string>());
	b.addExperimentParameter("nombre_del_archivo_del_entorno", "STRING", config["general/env_file"].as<std::string>());
	b.addExperimentParameter("nombre_del_archivo_del_robot", "STRING", config["general/robot_file"].as<std::string>());

    //std::vector<std::string> planners_v= config["general/planners"];
    for(int it = 0; it < config["general/planners"].size();it++) 
    {
	   std::string value = config["general/planners"][it].as<std::string>();

	   if (value.compare("DynamicRRT") == 0)
	   {
			b.addPlanner(base::PlannerPtr(new auvplanning::DynamicRRT(robot->getSimpleSetup().getSpaceInformation())));	
			OMPL_INFORM("Planificador DynamicRRT añadido al benchmark");	
	   }if (value.compare("RRT") == 0)
	   {
	   		b.addPlanner(base::PlannerPtr(new control::RRT(robot->getSimpleSetup().getSpaceInformation())));
	   		OMPL_INFORM("Planificador RRT añadido al benchmark");
	   }else if (value.compare("EST") == 0)
	   {
	   		for(int it_est = 0; it_est < config["EST/Range"].size();it_est++) 
	   		{
	   			double value_est = config["EST/Range"][it_est].as<double>();
	   			b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, getRobot(), value_est));
	   			b.addExperimentParameter("EST range " + it_est, "INTEGER", config["EST/Range"][it_est].as<std::string>());
	   			OMPL_INFORM("Planificador EST añadido al benchmark, range = %f",value_est);
	   		}
	   }else if (value.compare("KPIECE1") == 0)
	   {	   	
	   		
	   		for(int it_kpiece1 = 0; it_kpiece1 < config["KPIECE1/numPlanners"].as<double>();it_kpiece1++) 
	   		{
				double bf = config["KPIECE1/borderFraction"][it_kpiece1].as<double>();
				double goodCSF = config["KPIECE1/CellScoreFactor"][2*it_kpiece1].as<double>();
				double badCSF = config["KPIECE1/CellScoreFactor"][2*it_kpiece1+1].as<double>();
				int    mcsc = config["KPIECE1/MaxCloseSampleCount"][it_kpiece1].as<double>();
		   		b.addPlannerAllocator(std::bind(&myKPIECE1ConfiguredPlanner, getRobot(), bf, goodCSF, badCSF, mcsc));
		   		OMPL_INFORM("Planificador KPIECE1 añadido al benchmark");
	   		}
	   }else if (value.compare("PDST") == 0)
	   {	   	
			b.addPlannerAllocator(std::bind(&myPDSTConfiguredPlanner, getRobot()));	   		
	   		OMPL_INFORM("Planificador PDST añadido al benchmark");
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
    int res1 = chdir(folder_str.c_str());
    b.saveResultsToFile(config["benchmark/logFile"].as<std::string>().c_str());
    int res2 = chdir("..");
}

void Main::moveAUV()
{

	robot->setup(getTypeControlSampler(config["general/controlSampler"].as<std::string>()));
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

    control_->as<control::RealVectorControlSpace::ControlType>()->values[0] = t1;
    control_->as<control::RealVectorControlSpace::ControlType>()->values[1] = t2;
    control_->as<control::RealVectorControlSpace::ControlType>()->values[2] = t3;
	printf("control_ t1: %f\n", control_->as<control::RealVectorControlSpace::ControlType>()->values[0]);
	printf("control_ t2: %f\n", control_->as<control::RealVectorControlSpace::ControlType>()->values[1]);
	printf("control_ t3: %f\n", control_->as<control::RealVectorControlSpace::ControlType>()->values[2]);    

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

	robot->setup(getTypeControlSampler(config["general/controlSampler"].as<std::string>()));
	printf("[controlAUV] Init\n");

	base::State *state; 
	state = robot->getSimpleSetup().getSpaceInformation()->allocState();
	state->as<base::RealVectorStateSpace::StateType>()->values[0] = config["control/start"][0].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[1] = config["control/start"][1].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[2] = config["control/start"][2].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[3] = config["control/start"][3].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[4] = config["control/start"][4].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[5] = config["control/start"][5].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[6] = config["control/start"][6].as<double>();
	state->as<base::RealVectorStateSpace::StateType>()->values[7] = config["control/start"][7].as<double>();
	printf("Start state\n\t");
	robot->getSimpleSetup().getSpaceInformation()->printState(state);

	base::State *reference; 
	reference = robot->getSimpleSetup().getSpaceInformation()->allocState();
	reference->as<base::RealVectorStateSpace::StateType>()->values[0] = config["control/reference"][0].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[1] = config["control/reference"][1].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[2] = config["control/reference"][2].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[3] = config["control/reference"][3].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[4] = config["control/reference"][4].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[5] = config["control/reference"][5].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[6] = config["control/reference"][6].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[7] = config["control/reference"][7].as<double>();
	printf("Reference\n\t");
	robot->getSimpleSetup().getSpaceInformation()->printState(reference);

    controller::ControllerPtr controller(new controller::AUV2StepPID(robot->getSimpleSetup().getSpaceInformation().get()));

    //const unsigned int maxDuration = robot->getSimpleSetup().getSpaceInformation()->getMaxControlDuration();

    double res = robot->getSimpleSetup().getSpaceInformation()->getPropagationStepSize();
    const unsigned int duration = config["control/duration"].as<double>();
	int steps = (int)floor(0.5 + duration / res);

    printf("Duration: %d\n", duration);
	printf("[controlAUV] Start of controller\n");
    int tiempoEjecutado = controller->propagateController(state, reference, steps);
	printf("Result\n\t");
	robot->getSimpleSetup().getSpaceInformation()->printState(reference);
	printf("Tiempo ejecutado: %d\n", tiempoEjecutado);
	printf("[controlAUV] End of controller\n");
	printf("[controlAUV] -------------------\n");


	reference->as<base::RealVectorStateSpace::StateType>()->values[0] = config["control/reference"][0].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[1] = config["control/reference"][1].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[2] = config["control/reference"][2].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[3] = config["control/reference"][3].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[4] = config["control/reference"][4].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[5] = config["control/reference"][5].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[6] = config["control/reference"][6].as<double>();
	reference->as<base::RealVectorStateSpace::StateType>()->values[7] = config["control/reference"][7].as<double>();

	printf("[controlAUV] Control Sampler\n");
	ompl::control::DirectedControlSamplerPtr controlSampler_ = robot->getSimpleSetup().getSpaceInformation()->allocDirectedControlSampler();
	control::Control		*rctrl = robot->getSimpleSetup().getSpaceInformation()->allocControl();
	printf("Start state\n\t");
	robot->getSimpleSetup().getSpaceInformation()->printState(state);
	printf("Reference\n\t");
	robot->getSimpleSetup().getSpaceInformation()->printState(reference);
	int t = controlSampler_->sampleTo(rctrl, state, reference);
    printf("Duration: %d\n", t);
	printf("Result\n\t");
	robot->getSimpleSetup().getSpaceInformation()->printState(reference);

	robot->getSimpleSetup().getSpaceInformation()->freeState(state);
	robot->getSimpleSetup().getSpaceInformation()->freeState(reference);
	robot->getSimpleSetup().getSpaceInformation()->freeControl(rctrl);
}

void Main::checkState()
{
	robot->setup(getTypeControlSampler(config["general/controlSampler"].as<std::string>()));
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
	isValid? printf("Is valid\n"): printf("Is not valid\n");;
}

int Main::getTypeControlSampler(std::string string_value)
{
	int value = -1;
    value = (string_value.compare("Random") == 0)? AUV_SIMPLE_DCS : -1;
    value = (string_value.compare("SemiRandom") == 0)? AUV_SEMI_RANDOM_DCS : -1;
    value = (string_value.compare("PID") == 0)? AUV_PID_DCS : -1;
    value = (string_value.compare("2StepsPID") == 0)? AUV_2PID_DCS : -1;

    return value;	
}

void Main::printSolutionToFile(std::fstream &solutionFile, std::string quatFile_str, std::string controlsFile_str, bool modeController)
{
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

Main::Main(int typeOfStateSpace, YAML::Node configuration)
{
	oauv::AUVRobotPtr auxiliar(new oauv::AUVRobot(configuration, typeOfStateSpace));
	robot = auxiliar;
	config = configuration;
}

int main(int argc, char** argv){
	if (argc < 2)
	{
		printf("Please, select one available option. (-h for more information).\n");
	}else if(argc < 3)
	{

		if(strcmp(argv[1],"-h") == 0)
		{
			printf("Available options:\n");
			printf("-b : benchmark planners for one problem\n");
			printf("-p : plan a path from one state to another\n");
			printf("-d : move auv (for dynamics check)\n");
			printf("-c : control auv (for controller check)\n");
			printf("-v : validate one state in the enviroment\n");
			printf("-h : help\n");
		}else
		{
			printf("Please, introduce the configuration file.\n");
		}
	}else
	{

		YAML::Node config = YAML::LoadFile(argv[2]);

		std::string env_fname 	= config["general/env_file"].as<std::string>();
		std::string robot_fname = config["general/robot_file"].as<std::string>();

		//oauv::AUVRobotPtr robot(new oauv::AUVRobot(false,config));
		std::string distanceFunction = config["general/distanceFunction"].as<std::string>();
		int distanceFunctionType = -1;
		if(distanceFunction == "Euclidean")
		{
			distanceFunctionType = EUCLIDEAN_DISTANCE;
		}else if(distanceFunction == "Propagated")
		{
			distanceFunctionType = SIMPLE_PROPAGATE_DISTANCE;
		}else
		{
			printf("Please, select one of the available distance functions: euclidead or propagated.\n");
			return -1;
		}

		Main test(distanceFunctionType,config);

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
			printf("Unknown option.\n");
		}
	}

	return 0;
}
