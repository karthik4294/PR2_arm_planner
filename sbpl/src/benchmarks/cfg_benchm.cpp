#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/SimpleSetup.h>
#include <omplapp/apps/KinematicCarPlanning.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>

#include <sbpl/headers.h>
#include <sbpl/discrete_space_information/environment_ompl.h>
#include <sbpl/discrete_space_information/Chameleon.h>

int main(int argc, char *argv[])
{
	const char* envCfgFilename = argv[1];
	// ConfigFile cf(envCfgFilename);

	// //Read problem name, robot and environment mesh
 //    std::string name = cf.Value("problem","name");  
 //    std::string robot_name = cf.Value("problem","robot");
 //    std::string env_name = cf.Value("problem","world");

 //    //environment bounds
 //    double volume_min_x = cf.Value("problem","volume.min.x");
 //    double volume_min_y = cf.Value("problem","volume.min.y");
 //    double volume_max_x = cf.Value("problem","volume.max.x");
 //    double volume_max_y = cf.Value("problem","volume.max.y");

 //    //benchmark properties
 //    double time_limit = cf.Value("benchmark","time_limit");
 //    double mem_limit = cf.Value("benchmark","mem_limit");
 //    double run_count = cf.Value("benchmark","rount_count");
 //    std::string save_paths = cf.Value("benchmark","save_paths");

    double min_distance = 50;
    double act_distance;

	BenchmarkOptions bo(envCfgFilename);
	SE2Benchmark* b = new SE2Benchmark(bo);
	b->setup();

	boost::shared_ptr<ompl::base::SpaceInformation> spi = (b->setup_se2_)->getSpaceInformation();
	b->setup_se2_->setOptimizationObjectiveAndThreshold("length", std::numeric_limits<double>::max());
	ompl::base::StateSamplerPtr sampler_ = spi->allocStateSampler();

	ompl::base::ScopedState<ompl::base::SE2StateSpace> start(spi);
	ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(spi);

	std::string path = "../env_examples/ompl/Maze_planar/benchmark";
	std::string name = "Maze";
	std::string ext = ".cfg";
	std::string cfg_file;
	
	for(int i = 1; i <= 10; i++){

		bool set_startgoal = false;

		while(!set_startgoal){
			ompl::base::State *start_state = spi->allocState();
			ompl::base::State *goal_state = spi->allocState();
			sampler_->sampleUniform(start_state);
			sampler_->sampleUniform(goal_state);

			act_distance = b->setup_se2_->getStateSpace()->distance(start_state, goal_state);

			start = start_state;
			goal = goal_state;
			if(!(b->setup_se2_->getStateValidityChecker()->isValid(start_state))) {
				printf("start no valid\n");
				continue;
			}

			if(!(b->setup_se2_->getStateValidityChecker()->isValid(goal_state))){
				printf("goal not valid\n");
			continue;
			}

			if(act_distance > min_distance){
				b->setup_se2_->setStartAndGoalStates(start, goal, 0.01);
				set_startgoal = true;
				printf("A valid state\n");
			}
			else{
				continue;
			}
		}

		 std::ostringstream convert;
	     convert << i;
	     std::string out_num = convert.str();
	     cfg_file = path + name + out_num + ext;

	 	 std::ofstream out(cfg_file.c_str());
	     out << "[problem]\n";
	     out << "name = Maze\n";//ss<< name.c_str() << "\n";
	     out << "robot = car1_planar_robot.dae\n";//<< robot_name.c_str() << "\n";
	     out << "world = Maze_planar_env.dae\n";//<< env_name.c_str() << "\n";
	     out << "start.x = "<< start->getX()<<"\n";
	     out << "start.y = "<< start->getY()<<"\n";
	     out << "start.theta = "<< start->getYaw()<<"\n";
	     out << "goal.x = "<< goal->getX()<<"\n";
	     out << "goal.y = "<< goal->getY()<<"\n";
	     out << "goal.theta = "<< goal->getYaw()<<"\n";
	     out << "volume.min.x = -55.0\n";//<< volume_min_x <<"\n";
	     out << "volume.min.y = -55.0\n";//<< volume_min_y <<"\n";
	     out << "volume.max.x = 55.0\n";//<< volume_max_x <<"\n";
	     out << "volume.max.y = 55.0\n";//<< volume_max_y <<"\n";
	     out << "\n";
	     out << "[benchmark]\n";
	     out << "time_limit= 20.0\n";// << time_limit << "\n";
	     out << "mem_limit= 10000\n";// << mem_limit << "\n";
	     out << "run_count= 10\n";// << run_count << "\n";
	     out << "save_paths=shortest\n";// << save_paths.c_str() << "\n";
	     out << "\n";
	     out << "[planner]\n";
	     out << "rrt=\n";
	     out << "#lazyrrt=\n";
	     out << "rrtstar=\n";
	     out << "rrtconnect=\n";
	     out << "prmstar=\n";
	     out << "#kpiece=\n";
	     out << "#lbkpiece=\n";


         out.close();

	}


	return 0;
}
