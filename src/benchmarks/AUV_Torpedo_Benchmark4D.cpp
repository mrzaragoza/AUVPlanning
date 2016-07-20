//#include "kinematicTest.h"
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/control/planners/est/EST.h>
#include <omplapp/config.h>
#include <boost/math/constants/constants.hpp>
#include "robots/AUV_Torpedo4D.h"
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl/control/ControlSpace.h"

#include <stdio.h>  /* defines FILENAME_MAX */
	#ifdef WINDOWS
	    #include <direct.h>
	    #define GetCurrentDir _getcwd
	#else
	    #include <unistd.h>
	    #define GetCurrentDir getcwd
	 #endif

#include <iostream>
#include <fstream>

#include <string.h>

#include <fcl/collision.h>


using namespace ompl;

void AUV_TorpedoSetup(AUV_Torpedo& setup)
{
	// plan for kinematic car in SE(3)
	base::StateSpacePtr StSpace(setup.getStateSpace());
	
	// set the bounds for the R^3 part of SE(3)
	/*base::RealVectorBounds bounds(8);
	bounds.setLow(-1000.0);
	bounds.setHigh(1000.0);
	bounds.setLow(3,-M_PI);
	bounds.setHigh(3,M_PI);
	bounds.setLow(4,-3);
	bounds.setHigh(4,3);
	bounds.setLow(5,-3);
	bounds.setHigh(5,3);
	bounds.setLow(6,-3);
	bounds.setHigh(6,3);
	bounds.setLow(7,-3);
	bounds.setHigh(7,3);
	StSpace->as<base::RealVectorStateSpace>()->setBounds(bounds);*/

	// define start state
	base::ScopedState<base::RealVectorStateSpace> start(setup.getGeometricComponentStateSpace());
	/*start[0] = 350.;
	start[1] = 50.;
	start[2] = -991.;*/
	start[0] = 100.;
	start[1] = 50.;
	start[2] = 100.;
    start[3] = 0.;
    start[4] = 0.;
    start[5] = 0.;
    start[6] = 0.;
    start[7] = 0.;

	// define goal state
	base::ScopedState<base::RealVectorStateSpace> goal(setup.getGeometricComponentStateSpace());
	goal[0] = 655.;
	goal[1] = 1050.;
	goal[2] = 200.;
    goal[3] = 0.;
    goal[4] = 0.;
    goal[5] = 0.;
    goal[6] = 0.;
    goal[7] = 0.;



    //setup.getSpaceInformation()->printState(start);
	// set the start & goal states
	//setup.setStartAndGoalStates(start, goal, .1);
	setup.setStartAndGoalStates(
		setup.getFullStateFromGeometricComponent(start),
        setup.getFullStateFromGeometricComponent(goal), .1);


	setup.setup();

    setup.getSpaceInformation()->printProperties();
    setup.getSpaceInformation()->printSettings();
    start.print();

	printf ("FIN setup\n");
}


void AUV_TorpedoDemo(AUV_Torpedo& setup)
{
	setup.setPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
	//setup.setPlanner(base::PlannerPtr(new control::SyclopRRT(setup.getSpaceInformation())));

	//setup.setup();

	setup.getPlanner()->printProperties(std::cout);
	setup.getPlanner()->printSettings(std::cout);

	std::fstream benchmarkFile;
	std::ofstream benchmarkWithQuatFile;
  	benchmarkFile.open ("benchmarkDebug.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);

	std::cout << "D.1" << std::endl;
	// try to solve the problem
	if (setup.solve(50))
	{
		// print the (approximate) solution path: print states along the path
		// and controls required to get from one state to the next
		std::cout << "D.2" << std::endl;
		control::PathControl& path(setup.getSolutionPath());
		//path.interpolate(); // uncomment if you want to plot the path
		path.printAsMatrix(std::cout);
		path.printAsMatrix(benchmarkFile);
		path.interpolate(); // uncomment if you want to plot the path
		path.printAsMatrix(std::cout);
		if (!setup.haveExactSolutionPath())
		{
			std::cout << "Solution is approximate. Distance to actual goal is " <<
					setup.getProblemDefinition()->getSolutionDifference() << std::endl;
		}
	}
	std::cout << "D.3" << std::endl;

  	benchmarkWithQuatFile.open ("benchmarkWithQuatDebug.txt", std::fstream::out | std::fstream::trunc);

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
				benchmarkWithQuatFile << num;
				benchmarkWithQuatFile << " ";
				break;
			case 4:
			{
				fcl::Quaternion3f quat;
    			fcl::Vec3f zaxis(0., 0., 1.); //se pone así porque tiene que ser un vector unitario
    			quat.fromAxisAngle(zaxis, num);
				benchmarkWithQuatFile << quat.getW ();
				benchmarkWithQuatFile << " ";
				benchmarkWithQuatFile << quat.getX ();
				benchmarkWithQuatFile << " ";
				benchmarkWithQuatFile << quat.getY ();
				benchmarkWithQuatFile << " ";
				benchmarkWithQuatFile << quat.getZ ();
				benchmarkWithQuatFile << " ";
				break;
			}
			case 12:
				index = 0;
				benchmarkWithQuatFile << "\n";
				break;
		}		
	}

  	benchmarkFile.close();
  	benchmarkWithQuatFile.close();
}

void AUV_TorpedoBenchmark(AUV_Torpedo& setup)
{
	tools::Benchmark::Request request(50., 10000., 10); // runtime (s), memory (MB), run count

    //setup.setup ();

    tools::Benchmark b(setup, setup.getName());
    b.addPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
    b.benchmark(request);
    b.saveResultsToFile();
}

void moveAUV(AUV_Torpedo& setup, double t1, double t2, double t3, double tiempo){

	printf("[moveAUV] Init\n");
	setup.setup();
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
}



int main(int argc, char** argv)
{


	if (argc < 2){
		printf("Por favor, introduzca una de las opciones para ejecutar el programa.\n");
		return 0;
	}else{
		if(strcmp(argv[1],"-d") == 0){

			if(argc < 6){
				printf("Por favor, introduzca las tres actuaciones y el tiempo de duración para ejecutar la dinámica.\n");
				return 0;
			}

			double t1 = atoi(argv[2]);
			double t2 = atoi(argv[3]);
			double t3 = atoi(argv[4]);
			double tiempo = atoi(argv[5]);

			//if(argc == 7 && strcmp(argv[6],"-v") == 0) verbose = 1;

			AUV_Torpedo setup;
			moveAUV(setup, t1, t2, t3, tiempo);	

		}else if(strcmp(argv[1],"-p") == 0){

			if(argc < 3){
				printf("Por favor, introduzca el archivo de configuración a utilizar.\n");
				return 0;
			}

			const char* configFileName = argv[2];

			AUV_Torpedo setup;
			std::string env_fname =   "/home/guillermo/workspace_tfm/resources/entorno_columnas_x+90.dae";
			std::string robot_fname = "/home/guillermo/workspace_tfm/resources/cilinder_AUV.dae";
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
			
			AUV_Torpedo setup;
			std::string env_fname =   "/home/guillermo/workspace_tfm/resources/entorno_columnas_x+90.dae";
			std::string robot_fname = "/home/guillermo/workspace_tfm/resources/cilinder_AUV.dae";
		    setup.setEnvironmentMesh(env_fname.c_str());
			setup.setRobotMesh(robot_fname.c_str());
			AUV_TorpedoSetup(setup);
			AUV_TorpedoBenchmark(setup);

		}else if(strcmp(argv[1],"-s") == 0){

			if(argc < 3){
				printf("Por favor, introduzca las tres actuaciones para ejecutar la dinámica.\n");
				return 0;
			}

			AUV_Torpedo setup;
			std::string env_fname =   "/home/guillermo/workspace_tfm/resources/entorno_columnas_x+90.dae";
			std::string robot_fname = "/home/guillermo/workspace_tfm/resources/cilinder_AUV.dae";
		    setup.setEnvironmentMesh(env_fname.c_str());
			setup.setRobotMesh(robot_fname.c_str());
			AUV_TorpedoSetup(setup);

		}
	}

	return 0;



	 char cCurrentPath[FILENAME_MAX];

	 if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
	     {
	     return errno;
	     }

	cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */

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
	return 0;
}
