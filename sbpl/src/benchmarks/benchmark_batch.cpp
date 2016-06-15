#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <sbpl/headers.h>

int main(void)
{
	std::string command;
	std::string run_file = "./SE2benchmark ";
	std::string env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze1";
	std::string mot_prim = "../matlab/mprim/unicycle_alldirs_0_1.mprim ";
	std::string ext = ".cfg ";

	char* eps[] = {"1.5", "2", "3", "5", "10", "50"};

	std::string h_cfg_file;

	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon;
		     system(command.c_str());	
		}
	}
	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze2";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;

		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}

	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze3";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}

	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze4";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}

	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze5";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}

	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze6";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}

	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze7";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}

	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze8";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}

	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze9";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}


	env_path = "../env_examples/ompl/Maze_planar/benchmarkMaze10";
	for(int i = 0; i <= 0; i++){ 
	    for(int j = 1; j <= 1; j++){
	 		 std::ostringstream convert;
		     convert << j;
		     std::string out_num = convert.str();
		     h_cfg_file = env_path + ext;
		     std::string epsilon = eps[i];

		     command = run_file + h_cfg_file + mot_prim + epsilon ;
		     system(command.c_str());	
		}
	}

}