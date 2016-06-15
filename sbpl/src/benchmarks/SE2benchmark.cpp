#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <memory>

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

#include <sbpl/discrete_space_information/Chameleon.h>

using std::shared_ptr;

	double allocated_time_secs = 20.0; // in seconds
    double initialEpsilon = 1.5;
    bool bsearchuntilfirstsolution = false;
    bool bforwardsearch = true;

    bool set_startgoal = false;
    double min_distance = 20;
    double act_distance;

  	ReplanParams replan_params(allocated_time_secs);


// void preRunEvent(const ompl::base::PlannerPtr& planner, shared_ptr<PPMAPlanner> my_planner, MDPConfig *MDPCfg, shared_ptr<EnvironmentOMPL> environment, 
// 				char* envCfgFilename, char* motPrimFilename, const ompl::base::SpaceInformationPtr &si, SE2Benchmark* b)
// {	
// 	// if (my_planner->set_start(MDPCfg->startstateid) == 0) {
//  //        	printf("ERROR: failed to set start state\n");
//  //        	throw new SBPL_Exception();
//  //    	}
//  //    	if (my_planner->set_goal(MDPCfg->goalstateid) == 0) {
//  //        	printf("ERROR: failed to set goal state\n");
//  //      	throw new SBPL_Exception();
//  //    	}
// 	// return;
// 	const char* in_planner =  (planner->getName()).c_str();	
// 	const char* our_planner = "ppma_planner";

// 	if(!strcmp(in_planner,our_planner))
// 	{
// 		std::vector<sbpl_2Dpt_t> perimeterptsV;

// 		environment.reset(new EnvironmentOMPL(si));
// 		//my_planner.reset(new PPMAPlanner(si, environment.get(), bforwardsearch, allocated_time_secs, &replan_params));
		
// 		//Initialize environment
// 	    if (!environment->InitializeEnv(envCfgFilename, perimeterptsV, motPrimFilename)) {
// 	        printf("ERROR: InitializeEnv failed\n");
// 	        throw new SBPL_Exception();
// 	    }

// 	    // Initialize MDP Info
// 	    if (!environment->InitializeMDPCfg(MDPCfg)) {
// 	        printf("ERROR: InitializeMDPCfg failed\n");
// 	        throw new SBPL_Exception();
// 	    }

// 	    environment->cost_obj = ((b->setup_se2_)->getProblemDefinition())->getOptimizationObjective();

//     	if (my_planner->set_start(MDPCfg->startstateid) == 0) {
//         	printf("ERROR: failed to set start state\n");
//         	throw new SBPL_Exception();
//     	}
//     	if (my_planner->set_goal(MDPCfg->goalstateid) == 0) {
//         	printf("ERROR: failed to set goal state\n");
//       	throw new SBPL_Exception();
//     	}
// 	}
// }

// void postRunEvent(const ompl::base::PlannerPtr& planner, ompl::tools::Benchmark::RunProperties& run, EnvironmentOMPL* environment)
// {
// 	const char* in_planner =  (planner->getName()).c_str();	
// 	const char* our_planner = "ppma_planner";

// 	if(!strcmp(in_planner,our_planner))
// 	{
// 	    delete environment;	
// 	}
// }

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "SE2benchmark");

	char* envCfgFilename = argv[1];

	BenchmarkOptions bo(envCfgFilename);
	SE2Benchmark* b = new SE2Benchmark(bo);
	
	b->setup();
	//boost::shared_ptr<ompl::base::SpaceInformation> spi = (b->setup_se2_)->getSpaceInformation();

	// b->setup_se2_->setOptimizationObjectiveAndThreshold("length", std::numeric_limits<double>::max());
	// ompl::base::StateSamplerPtr sampler_ = spi->allocStateSampler();
	// spi->setStateValidityCheckingResolution(0.01);
	// spi->setMotionValidator(ompl::base::MotionValidatorPtr(new myMotionValidator(spi)));

    b->runBenchmark();	
    delete b;

	return 0;
}
