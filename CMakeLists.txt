#Compilar para debuguear: cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake_minimum_required (VERSION 2.8)
project (AUVPlanning)

set (AUVPlanning_VERSION_MAJOR 1)
set (AUVPlanning_VERSION_MINOR 0)

find_package(Boost COMPONENTS system REQUIRED )
find_package(OMPL REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)

set(FCL_LIBRARIES "/usr/local/lib/libfcl.so")
set(FCL_INCLUDE_DIRS "/usr/local/include")


MESSAGE( STATUS "OMPL LIBRARY DIRECTORY " ${OMPL_LIBRARIES} )
MESSAGE( STATUS "OMPL INCLUDE DIRECTORY " ${OMPL_INCLUDE_DIRS} )
MESSAGE( STATUS "OMPLAPP LIBRARY DIRECTORY " ${OMPLAPP_LIBRARIES} )
MESSAGE( STATUS "FCL LIBRARY DIRECTORY " ${FCL_LIBRARIES} )
MESSAGE( STATUS "FCL INCLUDE DIRECTORY " ${FCL_INCLUDE_DIRS} )
MESSAGE( STATUS "BOOST INCLUDE DIRECTORY " ${Boost_INCLUDE_DIRS} )
MESSAGE( STATUS "BOOST LIBRARY DIRECTORY " ${Boost_LIBRARIES} )

#if(CMAKE_COMPILER_IS_GNUCXX)
#    set(CMAKE_CXX_FLAGS "-O2")        ## Optimize
#    set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary
#endif()

set(CMAKE_CXX_FLAGS "-O4 -fpermissive -lompl -lompl_app_base -lboost_system -lyaml-cpp")

include_directories( includes ${OMPL_INCLUDE_DIRS} ${OMPLAPP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS} )
LINK_DIRECTORIES ( ${Boost_LIBRARY_DIRS} )

add_executable(AUVPlanning src/benchmarks/AUVPlanning.cpp 
	src/robots/AUVRobot.cpp
	src/robots/dynamics/AUVdynamics.cpp
	#src/robots/dynamics/ODEAUVSolver.cpp
	src/colisionador/RigidBodyGeometry.cpp 
	src/colisionador/appUtil.cpp
	#src/colisionador/assimpUtil.cpp
	src/planners/AUVGoal.cpp
	src/planners/PlanificadorLocal/MuestreadorControl/MuestreadorControl.cpp
	src/planners/PlanificadorLocal/AUVSemiRandomDirectedControlSampler.cpp 
	src/planners/PlanificadorLocal/AUVDirectedControlSampler.cpp
	src/planners/DynamicRRT.cpp
	src/planners/PlanificadorLocal/AUVPIDControlSampler.cpp
	src/planners/PlanificadorLocal/AUV2StepPIDControlSampler.cpp
	src/planners/PathController.cpp
	src/robots/control/Controller.cpp
	src/robots/control/AUVPID.cpp
	src/robots/control/AUV2StepPID.cpp
	)
set_property(TARGET AUVPlanning PROPERTY CXX_STANDARD 11)
target_link_libraries(AUVPlanning PUBLIC assimp ompl_app_base ompl_app yaml-cpp ${OMPLAPP_LIBRARIES} ${OMPL_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES})
MESSAGE(STATUS "target_link_libraries(AUVPlanning PUBLIC ${FCL_LIBRARIES} ${OMPLAPP_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})")

# g++ -o kinematicTest ../Kinematic2D.cpp ../kinematicTest.cpp -I/usr/local/include -L/usr/local/lib -lompl -lompl_app_base -lboost_system


# https://sourceforge.net/p/ompl/mailman/message/34464308/