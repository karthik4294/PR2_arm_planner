cmake_minimum_required(VERSION 2.8)
set(ROS_BUILD_TYPE Release)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
include_directories(${PROJECT_SOURCE_DIR}/code)
#PROJECT( sbpl )
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++0x")
# Add a ROS C++ macro to tell SBPL to use ROS print statements
add_definitions(-DROS)

find_package( PkgConfig REQUIRED)
find_package(OMPL)
find_package(OMPLAPP)

#pkg_check_modules( ompl REQUIRED ompl )
#pkg_check_modules( ompl_app REQUIRED ompl_app )
#pkg_check_modules( ompl_app_base REQUIRED ompl_app_base )
pkg_check_modules( gsl REQUIRED gsl )
rosbuild_add_boost_directories()
INCLUDE_DIRECTORIES(usr/local/include)
LINK_DIRECTORIES(usr/local/lib)

rosbuild_add_library(sbplros
  src/discrete_space_information/Chameleon.cpp
  src/discrete_space_information/environment_nav2D.cpp
  src/discrete_space_information/environment_navxythetalat.cpp
  src/discrete_space_information/environment_ompl.cpp
  src/discrete_space_information/environment_navxythetamlevlat.cpp
  src/discrete_space_information/environment_nav2Duu.cpp
  src/discrete_space_information/environment_XXX.cpp
  src/discrete_space_information/environment_robarm.cpp
  src/discrete_space_information/environment_ompl.cpp
  src/planners/araplanner.cpp
  src/planners/adplanner.cpp
  src/planners/viplanner.cpp
  src/planners/ppcpplanner.cpp
  src/planners/ANAplanner.cpp
  src/planners/rstarplanner.cpp
  src/planners/mha_planner.cpp
  src/planners/lazyARA.cpp
  src/planners/ppma.cpp
  #src/planners/ppma_control.cpp
  src/utils/heap.cpp
  src/utils/mdp.cpp
  src/utils/utils.cpp
  src/utils/2Dgridsearch.cpp
  src/utils/config.cpp 
  src/discrete_space_information/Chameleon.cpp
  )
#target_link_libraries(sbplros ${gsl_LIBRARIES} ompl ompl_app ompl_app_base boost_program_options) 
target_link_libraries(sbplros ${gsl_LIBRARIES} ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} ompl_app_base boost_program_options) 

#set(SBPL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
#set(SBPL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  
# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#rosbuild_add_executable(test_sbpl src/test/main.cpp)
#target_link_libraries(test_sbpl sbplros ompl ompl_app ompl_app_base boost_program_options)
#target_link_libraries(test_sbpl sbplros)

#rosbuild_add_executable(test_kinematic_car src/test/kinematic_car_test.cpp)
#target_link_libraries(test_kinematic_car sbplros ompl ompl_app ompl_app_base boost_program_options)
#target_link_libraries(test_kinematic_car sbplros)

#rosbuild_add_executable(ctrl_test_kinematic_car src/test/ctrl_kinematic_car_test.cpp)
#target_link_libraries(ctrl_test_kinematic_car sbplros ompl ompl_app ompl_app_base boost_program_options)
#target_link_libraries(ctrl_test_kinematic_car)

#rosbuild_add_executable(test_adjacency_list src/test/test_adjacency_list.cpp)
#target_link_libraries(test_adjacency_list sbplros ompl ompl_app ompl_app_base boost_program_options)
#target_link_libraries(test_adjacency_list sbplros)

#rosbuild_add_executable(SE2benchmark src/benchmarks/SE2benchmark.cpp)
#target_link_libraries(SE2benchmark sbplros ompl ompl_app ompl_app_base boost_program_options)

#rosbuild_add_executable(benchmark_batch src/benchmarks/benchmark_batch.cpp)
#target_link_libraries(benchmark_batch sbplros ompl ompl_app ompl_app_base boost_program_options)

#rosbuild_add_executable(cfg_benchm src/benchmarks/cfg_benchm.cpp)
#target_link_libraries(cfg_benchm sbplros ompl ompl_app ompl_app_base boost_program_options)

# Download needed data file
rosbuild_download_test_data(http://pr.willowgarage.com/data/sbpl/env_examples/nav2d/willow-25mm-inflated-env.cfg env_examples/nav2d/willow-25mm-inflated-env.cfg 1b4551eeba6df03b6e597e598bbaa4f3)
