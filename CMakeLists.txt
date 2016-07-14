#Compilar para debuguear: cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake_minimum_required (VERSION 2.8)
project (AUV_Torpedo_Benchmark)

set (AUV_Torpedo_Benchmark_VERSION_MAJOR 1)
set (AUV_Torpedo_Benchmark_VERSION_MINOR 0)

find_package(Boost COMPONENTS system REQUIRED )
#find_package(fcl REQUIRED)
find_package(OMPL REQUIRED)

#set(OMPL_LIBRARIES "/usr/local/lib")
#set(OMPLAPP_INCLUDE_DIRS "/usr/local/include/omplapp")

set(FCL_LIBRARIES "/usr/local/lib/libfcl.so")
set(FCL_INCLUDE_DIRS "/usr/local/include")

MESSAGE( STATUS "OMPL LIBRARY DIRECTORY " ${OMPL_LIBRARIES} )
MESSAGE( STATUS "OMPL INCLUDE DIRECTORY " ${OMPL_INCLUDE_DIRS} )
MESSAGE( STATUS "OMPLAPP LIBRARY DIRECTORY " ${OMPLAPP_LIBRARIES} )
MESSAGE( STATUS "FCL LIBRARY DIRECTORY " ${FCL_LIBRARIES} )
MESSAGE( STATUS "FCL INCLUDE DIRECTORY " ${FCL_INCLUDE_DIRS} )
MESSAGE( STATUS "BOOST INCLUDE DIRECTORY " ${Boost_INCLUDE_DIRS} )
MESSAGE( STATUS "BOOST LIBRARY DIRECTORY " ${Boost_LIBRARIES} )

set(CMAKE_CXX_FLAGS "-lompl -lompl_app_base -lboost_system")

include_directories( includes ${OMPL_INCLUDE_DIRS} ${OMPLAPP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS} )
LINK_DIRECTORIES ( ${Boost_LIBRARY_DIRS} )

add_executable(AUVTorpedoBenchmark src/benchmarks/AUV_Torpedo_Benchmark4D.cpp src/robots/AUV_Torpedo4D.cpp src/colisionador/RigidBodyGeometry.cpp src/colisionador/appUtil.cpp)
set_property(TARGET AUVTorpedoBenchmark PROPERTY CXX_STANDARD 11)
target_link_libraries(AUVTorpedoBenchmark PUBLIC assimp ompl_app_base ompl_app ${OMPLAPP_LIBRARIES} ${OMPL_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES})
MESSAGE(STATUS "target_link_libraries(AUVTorpedoBenchmark PUBLIC ${FCL_LIBRARIES} ${OMPLAPP_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})")


#add_executable(QuadrotorPlanning_Benchmark QuadrotorPlanning_Benchmark.cpp QuadrotorPlanning.cpp )

#target_link_libraries(QuadrotorPlanning_Benchmark PUBLIC ompl_app_base ompl_app ${OMPLAPP_LIBRARIES} ${OMPL_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES})
#MESSAGE(STATUS "target_link_libraries(QuadrotorPlanning_Benchmark PUBLIC ${FCL_LIBRARIES} ${OMPLAPP_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})")

#add_executable(AUV_Torpedo_Dinamica AUV_Torpedo_Dinamica.cpp)

#target_link_libraries(AUV_Torpedo_Dinamica PUBLIC ompl_app_base ompl_app ${OMPLAPP_LIBRARIES} ${OMPL_LIBRARIES} ${FCL_LIBRARIES} ${Boost_LIBRARIES})

# g++ -o kinematicTest ../Kinematic2D.cpp ../kinematicTest.cpp -I/usr/local/include -L/usr/local/lib -lompl -lompl_app_base -lboost_system


# https://sourceforge.net/p/ompl/mailman/message/34464308/