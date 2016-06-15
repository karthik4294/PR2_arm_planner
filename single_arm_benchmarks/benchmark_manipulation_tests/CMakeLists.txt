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

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosconsole genmsg)

rosbuild_init()
SET(CMAKE_CXX_FLAGS "-std=c++0x")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_find_ros_package(sbpl_geometry_utils)	
include_directories(include ${sbpl_geometry_utils_PACKAGE_PATH}/include)

#common commands for building c++ executables and libraries
rosbuild_add_library(benchmark_manipulation_tests src/benchmark_manipulation_tests.cpp)
target_link_libraries(benchmark_manipulation_tests sbpl_geometry_utils)

rosbuild_add_executable(runTest src/run_test.cpp)
target_link_libraries(runTest benchmark_manipulation_tests)

rosbuild_add_executable(visualizePaths src/visualize_paths.cpp)
target_link_libraries(visualizePaths benchmark_manipulation_tests)

#rosbuild_add_executable(envViz src/visualize_environment.cpp)
#target_link_libraries(envViz benchmark_manipulation_tests)
