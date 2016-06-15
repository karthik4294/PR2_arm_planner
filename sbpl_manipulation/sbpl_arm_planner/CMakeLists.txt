cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

# Add a ROS C++ macro to tell SBPL to use ROS print statements
add_definitions(-DROS)

find_package( PkgConfig REQUIRED)
find_package(OMPL)
find_package(OMPLAPP)

pkg_check_modules( gsl REQUIRED gsl )
rosbuild_add_boost_directories()
INCLUDE_DIRECTORIES(usr/local/include)
LINK_DIRECTORIES(usr/local/lib)

rosbuild_init()
SET(CMAKE_CXX_FLAGS "-std=c++0x")
rosbuild_add_boost_directories()

find_package(PkgConfig REQUIRED)
#pkg_check_modules(SBPL REQUIRED sbpl)
#include_directories(${SBPL_INCLUDE_DIRS})
#link_directories(${SBPL_LIBRARY_DIRS})

rosbuild_find_ros_package(sbpl_geometry_utils)
include_directories(include ${sbpl_geometry_utils_PACKAGE_PATH}/include)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#rosbuild_add_compile_flags(-g)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(sbpl_arm_planner 
                        src/environment_robarm3d.cpp
                        src/environment_omplrobarm3d.cpp
                        src/action_set.cpp
                        src/planning_params.cpp
                        src/sbpl_arm_planner_interface.cpp)

target_link_libraries(sbpl_arm_planner sbpl_manipulation_components leatherman bfs3d ${SBPL_LIBRARIES} ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} ompl_app_base boost_program_options)
