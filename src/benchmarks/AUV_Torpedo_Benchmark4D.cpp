#include <boost/math/constants/constants.hpp>
#include <omplapp/config.h>

#include <ompl/tools/benchmark/Benchmark.h>
//#include <planners/RRT.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/syclop/Syclop.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl/control/ControlSpace.h"
#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/SpaceInformation.h"
#include "planners/PlanificadorLocal/PlanificadorLocal.h"
#include "planners/AUVGoal.h"
#include "robots/AUV_Torpedo4D.h"

//#include <stdio.h>  /* defines FILENAME_MAX */
/*	#ifdef WINDOWS
	    #include <direct.h>
	    #define GetCurrentDir _getcwd
	#else
	    #include <unistd.h>
	    #define GetCurrentDir getcwd
	 #endif
*/
#include <iostream>
#include <fstream>

#include <string.h>

#include <fcl/collision.h>

#include "yaml-cpp/yaml.h"


using namespace ompl;

//const control::SpaceInformationPtr si;


void AUV_TorpedoSetup(AUV_Torpedo& setup, YAML::Node config)
{
	base::StateSpacePtr StSpace(setup.getStateSpace());

	// define start state
	base::ScopedState<base::RealVectorStateSpace> start(setup.getGeometricComponentStateSpace());
	start[0] = config["general/start"][0].as<double>();
	start[1] = config["general/start"][1].as<double>();
	start[2] = config["general/start"][2].as<double>();	
    /*start[3] = 0.;
    start[4] = 0.;
    start[5] = 0.;
    start[6] = 0.;
    start[7] = 0.;*/

	// define goal state
    base::ScopedState<base::RealVectorStateSpace> goal(setup.getGeometricComponentStateSpace());
    goal[0] = config["general/goal"][0].as<double>();
    goal[1] = config["general/goal"][1].as<double>();
    goal[2] = config["general/goal"][2].as<double>();
    /*goal[3] = 0.;
    goal[4] = 0.;
    goal[5] = 0.;
    goal[6] = 0.;
    goal[7] = 0.;*/

    setup.setStartAndGoalStates(
    	setup.getFullStateFromGeometricComponent(start),
    	setup.getFullStateFromGeometricComponent(goal), config["general/distanceToGoal"].as<double>());

	//si = setup.getSpaceInformation().get();
	/*ompl::control::DirectedControlSamplerAllocator pla = boost::bind(ompl::guillermo::PlanificadorLocal::PlanificadorLocalAllocator, setup.getSpaceInformation().get(), 15, start.get(), goal.get());
    setup.getSpaceInformation().get()->setDirectedControlSamplerAllocator(pla);*/

    setup.getSpaceInformation().get()->setPropagationStepSize(config["general/propStepSize"].as<double>());
    setup.getSpaceInformation().get()->setMinMaxControlDuration(config["general/minControlDuration"].as<double>(),config["general/maxControlDuration"].as<double>());

    //ompl::guillermo::PlanificadorLocal *pl = (ompl::guillermo::PlanificadorLocal *) setup.getSpaceInformation()->allocDirectedControlSampler().get();
    //pl->setGoal(goal.get());
    //pl->setStart(start.get());

    setup.getSpaceInformation()->setStateValidityCheckingResolution(config["general/checkresolution"].as<double>());


    setup.getStateSpace()->registerProjection("AUVProjection",allocGeometricStateProjector(setup.getStateSpace(),
    	guillermo::Motion_2_5D, setup.getGeometricComponentStateSpace(),
    	setup.getGeometricStateExtractor()));

    printf ("FIN setup\n");
}

ompl::base::PlannerPtr myESTConfiguredPlanner(AUV_Torpedo& setup, double range)
{
    ompl::control::EST *est = new ompl::control::EST(setup.getSpaceInformation());
    est->setRange(range);
    est->setProjectionEvaluator("AUVProjection");
    return ompl::base::PlannerPtr(est);
}


void AUV_TorpedoDemo(AUV_Torpedo& setup, YAML::Node config)
{

	std:string planner_str = config["plan/planner"].as<std::string>();

	if (planner_str.compare("RRT") == 0){
		setup.setPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));		
	}else if (planner_str.compare("EST") == 0){
		base::PlannerPtr planner = myESTConfiguredPlanner(setup, config["plan/ESTrange"].as<double>());
		setup.setPlanner(planner);
	}else if (planner_str.compare("KPIECE1") == 0){
		setup.setPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
	}else if (planner_str.compare("SyclopRRT") == 0){
		//setup.setPlanner(base::PlannerPtr(new control::SyclopRRT(setup.getSpaceInformation())));
	}else if (planner_str.compare("") == 0){
		
	}

	setup.setup();

	setup.getPlanner()->printProperties(std::cout);
	setup.getPlanner()->printSettings(std::cout);

	std::fstream benchmarkFile;
	std::ofstream benchmarkWithQuatFile;
	std::ofstream benchmarkControls;
  	benchmarkFile.open (config["plan/solutionFile"].as<std::string>(), std::fstream::in | std::fstream::out | std::fstream::trunc);

	// try to solve the problem
	if (setup.solve(config["plan/time"].as<double>()))
	{

		control::PathControl& path(setup.getSolutionPath());
		path.printAsMatrix(std::cout);
		path.interpolate(); // uncomment if you want to plot the path
		path.printAsMatrix(benchmarkFile);
		if (!setup.haveExactSolutionPath())
		{
			std::cout << "Solution is approximate. Distance to actual goal is " <<
					setup.getProblemDefinition()->getSolutionDifference() << std::endl;
		}else{
			std::cout << "Solution is in the range. Distance to actual goal is " <<
					setup.getProblemDefinition()->getSolutionDifference() << std::endl;
		}
	}

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

void AUV_TorpedoBenchmark(AUV_Torpedo& setup, YAML::Node config)
{

	double runtime = config["benchmark/runtime"].as<double>();
	double runmemory = config["benchmark/runmemory"].as<double>();
	double runcount = config["benchmark/runcount"].as<double>();

	tools::Benchmark::Request request(runtime, runmemory, runcount); 
	request.displayProgress = config["benchmark/displayProgress"].as<bool>();
	request.useThreads = config["benchmark/useThreads"].as<bool>();

    setup.setup();

    tools::Benchmark b(setup, config["benchmark/testname"].as<std::string>());
    //b.addExperimentParameter("save_paths", "INTEGER", "10")
    //b.addPlanner(base::PlannerPtr(new guillermo::RRT(setup.getSpaceInformation())));
    /*b.addPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::EST(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::PDST(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));*/
	//b.addPlanner(base::PlannerPtr(new control::Syclop(setup.getSpaceInformation())));
	//b.addPlanner(base::PlannerPtr(new control::SyclopRRT(setup.getSpaceInformation())));
	//b.addPlanner(base::PlannerPtr(new control::SyclopEST(setup.getSpaceInformation())));
	//b.addPlanner(base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));

    //std::vector<std::string> planners_v= config["benchmark/planners"];
    for(int it = 0; it < config["benchmark/planners"].size();it++) {

	   std::string value = config["benchmark/planners"][it].as<std::string>();

	   if (value.compare("RRT") == 0)
	   {
	   		b.addPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
	   		printf("Planificador RRT añadido al benchmark\n");
	   }else if (value.compare("EST") == 0)
	   {
	   		for(int it_est = 0; it_est < config["benchmark/ESTrange"].size();it_est++) {
	   			double value_est = config["benchmark/ESTrange"][it_est].as<double>();
	   			b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,value_est));
	   			printf("Planificador EST añadido al benchmark, range = %f\n",value_est);
	   		}
	   }else if (value.compare("KPIECE1") == 0)
	   {
	   		b.addPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
	   		printf("Planificador KPIECE1 añadido al benchmark\n");
	   }else if (value.compare("PDST") == 0)
	   {
	   		b.addPlanner(base::PlannerPtr(new control::PDST(setup.getSpaceInformation())));
	   		printf("Planificador PDST añadido al benchmark\n");
	   }else if (value.compare("SyclopRRT") == 0)
	   {
	   		
	   }else if (value.compare("SyclopEST") == 0)
	   {
	   		
	   }
	   
	}

    b.benchmark(request);
    b.saveResultsToFile(config["benchmark/resultsFile"].as<std::string>().c_str());
}

void moveAUV(AUV_Torpedo& setup, YAML::Node config){

	setup.setup();
	double t1 = config["dynamics/th1"].as<double>();
	double t2 = config["dynamics/th2"].as<double>();
	double t3 = config["dynamics/th3"].as<double>();
	double tiempo = config["dynamics/time"].as<double>();
	printf("[moveAUV] Init\n");

	base::State *state; 
	state = setup.getSpaceInformation()->allocState();
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
    setup.getStateSpace()->copyToReals(reals, state); 
    printf("size of reals: %d\n", static_cast<int>(reals.size())); 

    ompl::control::Control *control_ = setup.getSpaceInformation()->allocControl();
    control_->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = t1;
    control_->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] = t2;
    control_->as<ompl::control::RealVectorControlSpace::ControlType>()->values[2] = t3;
	printf("control_ t1: %f\n", control_->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0]);
	printf("control_ t2: %f\n", control_->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1]);
	printf("control_ t3: %f\n", control_->as<ompl::control::RealVectorControlSpace::ControlType>()->values[2]);

    base::State *result = setup.getSpaceInformation()->allocState();

    control::StatePropagatorPtr stProp = setup.getSpaceInformation()->getStatePropagator();

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

	ompl::control::DirectedControlSamplerPtr controlSampler_ = setup.getSpaceInformation()->allocDirectedControlSampler();

	//controlSampler_->as<SimpleDirectedControlSampler>()->setNumControlSamples(20);

	ompl::control::Control *rctrl = setup.getSpaceInformation()->allocControl();
	unsigned int cd = controlSampler_->sampleTo(rctrl, state, result);
	printf("rctrl t1: %f\n", rctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0]);
	printf("rctrl t2: %f\n", rctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1]);
	printf("rctrl t3: %f\n", rctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[2]);
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

		AUV_Torpedo setup;

		if(strcmp(argv[1],"-d") == 0){
			AUV_TorpedoSetup(setup,config);
			moveAUV(setup, config);	

		}else if(strcmp(argv[1],"-p") == 0){

			if(argc < 3){
				printf("Por favor, introduzca el archivo de configuración a utilizar.\n");
				return 0;
			}

			const char* configFileName = argv[2];

		    setup.setEnvironmentMesh(env_fname.c_str());
			setup.setRobotMesh(robot_fname.c_str());
			AUV_TorpedoSetup(setup,config);
			AUV_TorpedoDemo(setup,config);

		}else if(strcmp(argv[1],"-b") == 0){

			if(argc < 3){
				printf("Por favor, introduzca el archivo de configuración a utilizar.\n");				
				return 0;
			}

			const char* configFileName = argv[2];
			
		    setup.setEnvironmentMesh(env_fname.c_str());
			setup.setRobotMesh(robot_fname.c_str());
			AUV_TorpedoSetup(setup,config);
			AUV_TorpedoBenchmark(setup,config);

		}else if(strcmp(argv[1],"-s") == 0){

			if(argc < 3){
				printf("Por favor, introduzca las tres actuaciones para ejecutar la dinámica.\n");
				return 0;
			}

		    setup.setEnvironmentMesh(env_fname.c_str());
			setup.setRobotMesh(robot_fname.c_str());
			AUV_TorpedoSetup(setup,config);

		}else{
			printf("Opción desconocida.\n");
			return 0;
		}
	}

	return 0;

}
