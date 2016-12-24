#ifndef AUVPLANNING_H_
#define AUVPLANNING_H_

#include <boost/math/constants/constants.hpp>
#include <omplapp/config.h>

#include <ompl/tools/benchmark/Benchmark.h>
//#include <planners/RRT.h>
#include "planners/DynamicRRT.h"
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
#include "ompl/control/SpaceInformation.h"

#include "planners/AUVGoal.h"

#include <robots/AUVRobot.h>
#include <colisionador/appUtil.h>
#include <omplapp/config.h>
#include <boost/math/constants/constants.hpp>

// FCL Headers
#include <fcl/collision.h>
#include <fcl/collision_node.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/continuous_collision.h>

#include <ompl/control/ODESolver.h>

#include <iostream>
#include <fstream>

#include <string.h>
#include <string>

#include "yaml-cpp/yaml.h"

#include "robots/control/Controller.h"
#include "robots/control/AUVPID.h"
#include "robots/control/AUV2StepPID.h"
#include "planners/PathController.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

class Main
{
    public:

    	Main(YAML::Node configuration);
    	~Main(){}
    	void AUVRobotSetup();
		void AUVRobotDemo();
        void AUVRobotBenchmark();
        void moveAUV();
        void controlAUV();
        void checkState();

        ompl::auvplanning::AUVRobotPtr getRobot(){ return robot;}
        static int getRunCounter(){ return runCounter;}
        static YAML::Node getConfig(){ return config;}
        static void addRun(){ runCounter++;}
        static void printSolutionToFile(std::fstream &solutionFile, std::string quatFile_str, std::string controlsFile_str, bool modeController);
        static int runCounter;
        static YAML::Node config;

    private:
        /*void optionalPreRunEvent(const base::PlannerPtr &planner);
        void optionalPostRunEvent(const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run);*/

        int getTypeControlSampler(std::string string_value);

        /*base::PlannerPtr myKPIECE1ConfiguredPlanner(int a);
        base::PlannerPtr myESTConfiguredPlanner(double range);*/

        ompl::auvplanning::AUVRobotPtr robot;
        /*int runCounter = 0;
        YAML::Node config;*/
};

#endif /* AUVPLANNING_H_ */
