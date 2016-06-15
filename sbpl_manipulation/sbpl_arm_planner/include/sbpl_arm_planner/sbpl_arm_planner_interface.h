/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Benjamin Cohen */

#ifndef _SBPL_ARM_PLANNER_INTERFACE_H_
#define _SBPL_ARM_PLANNER_INTERFACE_H_

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <kdl/frames.hpp>
#include <angles/angles.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>
#include <leatherman/viz.h>
#include <sbpl/planners/araplanner.h>
#include <sbpl/planners/ppma.h>
#include <sbpl/planners/mha_planner.h>  //fahad
#include <sbpl/planners/planner.h>
#include <sbpl_arm_planner/environment_omplrobarm3d.h>
#include <sbpl_manipulation_components/post_processing.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <geometry_msgs/Pose.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <trajectory_msgs/JointTrajectory.h> 
#include <sbpl/headers.h>

 /** OMPL **/
//#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "ompl/base/goals/GoalState.h"

namespace sbpl_arm_planner{

class SBPLArmPlannerInterface
{
  public:
    
    SBPLArmPlannerInterface(RobotModel *rmodel, CollisionChecker *cc, ActionSet* as, distance_field::PropagationDistanceField* df);
    SBPLArmPlannerInterface(const ompl::base::SpaceInformationPtr &si, RobotModel *rm, CollisionChecker *cc, ActionSet* as, distance_field::PropagationDistanceField* df);

    ~SBPLArmPlannerInterface();

    bool init(std::string ns="~");

    bool getParams();

    bool planKinematicPath(const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res);

    bool solve(const arm_navigation_msgs::PlanningSceneConstPtr& planning_scene, const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res, std::vector<trajectory_msgs::JointTrajectoryPoint> &straj, vector<double> full_start, vector<double> full_goal, double &simplification_time);

    bool canServiceRequest(const arm_navigation_msgs::GetMotionPlan::Request &req);

    std::map<std::string, double>  getPlannerStats();

    visualization_msgs::MarkerArray getVisualization(std::string type);

    visualization_msgs::MarkerArray getCollisionModelTrajectoryMarker();

    void setStartAndGoal(const std::vector<double> &start_angles, const std::vector<double> &goal_angles);

    void visualizeOMPLpath(const trajectory_msgs::JointTrajectory &traj);

  private:

    ros::NodeHandle nh_;

    /* params */
    bool planner_initialized_;
    int num_joints_;
    int solution_cost_;
    int epsilon_;

    bool backward_forward;
    vector<double> start_full;
    vector<double> goal_full;

    /* planner & environment */
    MDPConfig mdp_cfg_;
    PPMAPlanner* planner_;
    MHAPlanner* mha_planner_;
    sbpl_arm_planner::EnvironmentOMPLROBARM3D *sbpl_arm_env_;
    sbpl_arm_planner::CollisionChecker *cc_;
    sbpl_arm_planner::OccupancyGrid *grid_;
    sbpl_arm_planner::RobotModel *rm_;
    sbpl_arm_planner::ActionSet *as_;
    sbpl_arm_planner::PlanningParams *prm_;
    distance_field::PropagationDistanceField* df_;

    arm_navigation_msgs::MotionPlanRequest req_;
    arm_navigation_msgs::GetMotionPlan::Response res_;
    arm_navigation_msgs::PlanningScene pscene_;

    /** \brief Initialize the SBPL planner and the sbpl_arm_planner environment */
    bool initializePlannerAndEnvironment(std::string ns="~");

    /** \brief Set start configuration */
    bool setStart(const sensor_msgs::JointState &state);

    bool setGoal(vector<double> initial_positions);     //fahad

    /** \brief Set goal(s) */
    bool setGoalPosition(const arm_navigation_msgs::Constraints &goals);

    bool setStartPosition(const arm_navigation_msgs::Constraints &goals, vector<double> start_full);        //fahad

    /** \brief Plan a path to a cartesian goal(s) */
    bool planToPosition(const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res, std::vector<trajectory_msgs::JointTrajectoryPoint> &straj, double &simplification_time);

    /** \brief Retrieve plan from sbpl */
    bool plan(trajectory_msgs::JointTrajectory &traj, std::vector<trajectory_msgs::JointTrajectoryPoint> &straj, double &totalTime, double &simplification_time);
    void packageMHAStats(vector<string> &stat_names,       //fahad
    vector<double> &stats,
    int solution_cost,
    size_t solution_size);

    //ompl space pointer
    const ompl::base::SpaceInformationPtr si_;
    //replan params
    ReplanParams replan_params;
    double allocated_time_secs;
    bool bforwardsearch;

    ompl::base::ProblemDefinitionPtr pdef;
    ompl::base::PathPtr ret_path, ret_path2;
    boost::shared_ptr<ompl::geometric::PathGeometric> ret_path_g;
    ompl::geometric::PathSimplifier* pathSimplifier;
};

}

#endif
