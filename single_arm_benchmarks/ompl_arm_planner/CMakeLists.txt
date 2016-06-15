cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "-std=c++0x")


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# Compiling with system dependency
find_package(OMPL)
find_package(OMPLAPP)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

#rosbuild_find_ros_package(sbpl_geometry_utils)
#include_directories(include ${sbpl_geometry_utils_PACKAGE_PATH}/include)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/ompl_arm_planner_interface.cpp)
target_link_libraries(${PROJECT_NAME} sbpl_manipulation_components ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} ompl_app_base)

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
