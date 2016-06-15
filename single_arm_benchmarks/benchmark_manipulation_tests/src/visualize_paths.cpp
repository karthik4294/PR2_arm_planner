#include <benchmark_manipulation_tests/benchmark_manipulation_tests.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "visualize_paths");
  BenchmarkManipulationTests exp;
  
  if(!exp.getParams())
  {
    ROS_ERROR("Failed to get all required params from param server.");
    return false;
  }
  exp.printParams();

  sleep(2);
  exp.visualizeEnvironment();
  ros::spinOnce();
  
  sleep(2);
  exp.visualizePaths();
  ros::spinOnce();
  
  return 0;
}

             
