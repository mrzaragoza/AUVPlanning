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


using namespace ompl;

//const control::SpaceInformationPtr si;

void AUV_TorpedoSetup(AUV_Torpedo& setup)
{
	base::StateSpacePtr StSpace(setup.getStateSpace());

	// define start state
	base::ScopedState<base::RealVectorStateSpace> start(setup.getGeometricComponentStateSpace());
	start[0] = 600.;
	start[1] = 120.;
	start[2] = 100.;
    /*start[3] = 0.;
    start[4] = 0.;
    start[5] = 0.;
    start[6] = 0.;
    start[7] = 0.;*/

	// define goal state
	base::ScopedState<base::RealVectorStateSpace> goal(setup.getGeometricComponentStateSpace());
	goal[0] = 550.;
	goal[1] = 330.;
	goal[2] = 115.;
    /*goal[3] = 0.;
    goal[4] = 0.;
    goal[5] = 0.;
    goal[6] = 0.;
    goal[7] = 0.;*/

	setup.setStartAndGoalStates(
		setup.getFullStateFromGeometricComponent(start),
        setup.getFullStateFromGeometricComponent(goal), 3.0);

	//si = setup.getSpaceInformation().get();
	/*ompl::control::DirectedControlSamplerAllocator pla = boost::bind(ompl::guillermo::PlanificadorLocal::PlanificadorLocalAllocator, setup.getSpaceInformation().get(), 15, start.get(), goal.get());
    setup.getSpaceInformation().get()->setDirectedControlSamplerAllocator(pla);*/

    setup.getSpaceInformation().get()->setPropagationStepSize(0.1);
    setup.getSpaceInformation().get()->setMinMaxControlDuration(25,500);

    //ompl::guillermo::PlanificadorLocal *pl = (ompl::guillermo::PlanificadorLocal *) setup.getSpaceInformation()->allocDirectedControlSampler().get();
    //pl->setGoal(goal.get());
    //pl->setStart(start.get());

    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

	printf ("FIN setup\n");
}


void AUV_TorpedoDemo(AUV_Torpedo& setup)
{
	base::PlannerPtr planner (new control::EST(setup.getSpaceInformation()));
	planner->as<ompl::control::EST>()->setRange(500);
	setup.setPlanner(planner);
	//setup.setPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
	//setup.setPlanner(base::PlannerPtr(new guillermo::RRT(setup.getSpaceInformation())));
	//setup.setPlanner(base::PlannerPtr(new control::SyclopRRT(setup.getSpaceInformation())));

	setup.setup();

	setup.getPlanner()->printProperties(std::cout);
	setup.getPlanner()->printSettings(std::cout);

	std::fstream benchmarkFile;
	std::ofstream benchmarkWithQuatFile;
	std::ofstream benchmarkControls;
  	benchmarkFile.open ("benchmarkDebug.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);

	// try to solve the problem
	if (setup.solve(150))
	{
		// print the (approximate) solution path: print states along the path
		// and controls required to get from one state to the next

		control::PathControl& path(setup.getSolutionPath());
		//path.interpolate(); // uncomment if you want to plot the path
		path.printAsMatrix(std::cout);
		path.interpolate(); // uncomment if you want to plot the path
		path.printAsMatrix(benchmarkFile);
		//path.printAsMatrix(std::cout);
		if (!setup.haveExactSolutionPath())
		{
			std::cout << "Solution is approximate. Distance to actual goal is " <<
					setup.getProblemDefinition()->getSolutionDifference() << std::endl;
		}else{
			std::cout << "Solution is in the range. Distance to actual goal is " <<
					setup.getProblemDefinition()->getSolutionDifference() << std::endl;
		}
	}

  	benchmarkWithQuatFile.open ("benchmarkWithQuatDebug.txt", std::fstream::out | std::fstream::trunc);
  	benchmarkControls.open ("benchmarkControls.txt", std::fstream::out | std::fstream::trunc);

    benchmarkFile.seekg (0, ios::beg);
    double num = 0.0;
    int index = 0;
	while(benchmarkFile >> num){
		index++;
		//std::cout << index << " " << num << std::endl;

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


ompl::base::PlannerPtr myESTConfiguredPlanner(AUV_Torpedo& setup, double range)
{
    ompl::control::EST *est = new ompl::control::EST(setup.getSpaceInformation());
    est->setRange(range);
    return ompl::base::PlannerPtr(est);
}



void AUV_TorpedoBenchmark(AUV_Torpedo& setup)
{
	tools::Benchmark::Request request(180., 10000., 1); // runtime (s), memory (MB), run count
	//tools::Benchmark::Request request(100., 10000., 1); // runtime (s), memory (MB), run count
	request.displayProgress = true;
	request.useThreads = true;


    setup.setup ();

    tools::Benchmark b(setup, "ESTtest"/*setup.getName()*/);
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



	b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,10));
	b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,50));
	b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,75));
	b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,100));
	b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,250));
	b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,500));
	b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,750));
	b.addPlannerAllocator(std::bind(&myESTConfiguredPlanner, setup ,1000));
    b.benchmark(request);
    b.saveResultsToFile();
}

void moveAUV(AUV_Torpedo& setup, double t1, double t2, double t3, double tiempo){

	printf("[moveAUV] Init\n");
	//setup.setup();
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
	}else{

		//std::string env_fname =   "/home/guillermo/workspace_tfm/resources/entorno_columnas_x+90.dae";
		std::string env_fname =   "/home/guillermo/workspace_tfm/resources/entorno_vacio_x+90.dae";
		//std::string robot_fname = "/home/guillermo/workspace_tfm/resources/cilinder_AUV.dae";
		std::string robot_fname = "/home/guillermo/workspace_tfm/resources/freeCAD/cilindro_0_1.dae";
		AUV_Torpedo setup;

		if(strcmp(argv[1],"-d") == 0){

			if(argc < 6){
				printf("Por favor, introduzca las tres actuaciones y el tiempo de duración para ejecutar la dinámica.\n");
				return 0;
			}

			double t1 = atof(argv[2]);
			double t2 = atof(argv[3]);
			double t3 = atof(argv[4]);
			double tiempo = atof(argv[5]);

			//if(argc == 7 && strcmp(argv[6],"-v") == 0) verbose = 1;

			//AUV_TorpedoSetup(setup);
			setup.setup();
			moveAUV(setup, t1, t2, t3, tiempo);	

		}else if(strcmp(argv[1],"-p") == 0){

			if(argc < 3){
				printf("Por favor, introduzca el archivo de configuración a utilizar.\n");
				return 0;
			}

			const char* configFileName = argv[2];

		    setup.setEnvironmentMesh(env_fname.c_str());
			setup.setRobotMesh(robot_fname.c_str());
			AUV_TorpedoSetup(setup);
			AUV_TorpedoDemo(setup);

		}else if(strcmp(argv[1],"-b") == 0){

			if(argc < 3){
				printf("Por favor, introduzca el archivo de configuración a utilizar.\n");				
				return 0;
			}

			const char* configFileName = argv[2];
			
		    setup.setEnvironmentMesh(env_fname.c_str());
			setup.setRobotMesh(robot_fname.c_str());
			AUV_TorpedoSetup(setup);
			AUV_TorpedoBenchmark(setup);

		}else if(strcmp(argv[1],"-s") == 0){

			if(argc < 3){
				printf("Por favor, introduzca las tres actuaciones para ejecutar la dinámica.\n");
				return 0;
			}

		    setup.setEnvironmentMesh(env_fname.c_str());
			setup.setRobotMesh(robot_fname.c_str());
			AUV_TorpedoSetup(setup);

		}else{
			printf("Opción desconocida.\n");
			return 0;
		}
	}

	return 0;

/*

	 char cCurrentPath[FILENAME_MAX];

	 if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
	     {
	     return errno;
	     }

	cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
/*
	printf ("The current working directory is %s\n", cCurrentPath);
	printf ("The OMPL RESOURCE directory is %s\n", std::string(OMPLAPP_RESOURCE_DIR).c_str());

	AUV_Torpedo setup;
	std::string env_fname =   "/home/guillermo/workspace_tfm/resources/entorno_vacio.dae";
	std::string robot_fname = "/home/guillermo/workspace_tfm/resources/cilinder_AUV.dae";
    setup.setEnvironmentMesh(env_fname.c_str());
	setup.setRobotMesh(robot_fname.c_str());
	//std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    //std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";


	//moveAUV(setup);


	AUV_TorpedoSetup(setup);

	AUV_TorpedoDemo(setup);
	//AUV_TorpedoBenchmark(setup);
	return 0;*/
}
