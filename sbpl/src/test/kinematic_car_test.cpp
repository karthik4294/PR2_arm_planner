/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#include <omplapp/apps/KinematicCarPlanning.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>

using namespace std;

#include <sbpl/headers.h>

int planxythetalat(char* envCfgFilename, char* eps, char* motPrimFilename)
{
    ompl::app::SE2RigidBodyPlanning setup;

    int bRet = 0;
    double allocated_time_secs = 100.0; // in seconds
    double initialEpsilon = 8;//atof(eps);
    MDPConfig MDPCfg;
    bool bsearchuntilfirstsolution = false;
    bool bforwardsearch = true;

    // set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
    vector<sbpl_2Dpt_t> perimeterptsV;
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.01; //0.3;
    double halflength = 0.01; //0.45;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);

    // clear the footprint
    perimeterptsV.clear();

    //Get space information
    ompl::base::SpaceInformationPtr si = setup.getSpaceInformation();

    // Initialize Environment (should be called before initializing anything else)
    EnvironmentOMPL environment_navxythetalat(si);

    if (!environment_navxythetalat.InitializeEnv(envCfgFilename, perimeterptsV, motPrimFilename)) {
        printf("ERROR: InitializeEnv failed\n");
        throw new SBPL_Exception();
    }

    // Initialize MDP Info
    if (!environment_navxythetalat.InitializeMDPCfg(&MDPCfg)) {
        printf("ERROR: InitializeMDPCfg failed\n");
        throw new SBPL_Exception();
    }

    const EnvOMPLConfig_t* env_cfg = environment_navxythetalat.GetEnvNavConfig();

    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(si->getStateSpace());
    start->setX(env_cfg->start_x);
    start->setY(env_cfg->start_y);
    start->setYaw(env_cfg->start_theta);

    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(si->getStateSpace());
    goal->setX(env_cfg->goal_x);
    goal->setY(env_cfg->goal_y);
    goal->setYaw(env_cfg->goal_theta);
    
    setup.setStartAndGoalStates(start, goal, .1);

    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, env_cfg->volume_min_x);
    bounds.setLow(1, env_cfg->volume_min_y);
    bounds.setHigh(0, env_cfg->volume_max_x);
    bounds.setHigh(1, env_cfg->volume_max_y);
    si->getStateSpace()->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    std::string robot_fname = environment_navxythetalat.env_path + env_cfg->robot_name;
    setup.setRobotMesh(robot_fname.c_str());
    std::string env_fname = environment_navxythetalat.env_path + env_cfg->env_name;    
    setup.setEnvironmentMesh(env_fname.c_str());

    //setup.setOptimizationObjectiveAndThreshold("length", std::numeric_limits<double>::max()); 

    si->setStateValidityCheckingResolution(0.01);

    si->setMotionValidator(ompl::base::MotionValidatorPtr(new myMotionValidator(si)));

    setup.setup();

    //si->printSettings();

    ompl::base::ProblemDefinitionPtr pdef = setup.getProblemDefinition();
    environment_navxythetalat.cost_obj = setup. getProblemDefinition()->getOptimizationObjective();

    //Publish robot and env in rviz
    environment_navxythetalat.publish_env_rob();

    // plan a path
    vector<int> solution_stateIDs_V;
    //planner->set_initialsolution_eps(initialEpsilon);
    // planner->set_search_mode(bsearchuntilfirstsolution);
    ReplanParams replan_params(allocated_time_secs);
    replan_params.initial_eps = initialEpsilon;
    replan_params.final_eps = initialEpsilon;
    replan_params.return_first_solution = bsearchuntilfirstsolution;

    ros::NodeHandle nh("~");
    int h_star_mode = 0;
    nh.param("h_star_mode", h_star_mode, 0);

    PPMAPlanner* planner = new PPMAPlanner(si, &environment_navxythetalat, bforwardsearch, allocated_time_secs, &replan_params);
    planner->planner_mode_ = static_cast<PlannerMode>(h_star_mode);
    planner->setProblemDefinition(ompl::base::ProblemDefinitionPtr(pdef));

    // set planner properties
    if (planner->set_start(MDPCfg.startstateid) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }

    // plan
    printf("start planning...\n");
    bRet = planner->replan(&(planner->solution_stateIDs_V),planner->replan_params);
    printf("done planning\n");
    printf("size of solution=%d\n", (unsigned int)solution_stateIDs_V.size());

    visualization_msgs::MarkerArray display_rviz = environment_navxythetalat.seeinRviz();
    (environment_navxythetalat.marker_array_pub).publish(display_rviz);

    environment_navxythetalat.PrintTimeStat(stdout);

    //getting the heuristic grid
    environment_navxythetalat.PrintHeuristicValues();

    // environment_navxythetalat.PrintTimeStat(stdout);

    // print a path
    if (bRet) {
        // print the solution
        printf("Solution is found\n");
    }
    else {
        printf("Solution does not exist\n");
    }

    fflush(NULL);

    delete planner;

    return bRet;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kinematic_car_test");
    char* motPrimFilename = argv[2];
    const int plannerRes = planxythetalat(argv[1], argv[3], motPrimFilename);

    return plannerRes;
}
