/*fahad
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <visualization_msgs/Marker.h>
#include <egraphs/egraph_stat_writer.h>

#include <chrono>

 #define ROS

clock_t starttime;

using namespace sbpl_arm_planner;

bool ppma = true;

SBPLArmPlannerInterface::SBPLArmPlannerInterface(RobotModel *rm, CollisionChecker *cc, ActionSet* as, distance_field::PropagationDistanceField* df): 
  nh_("~"), planner_(NULL), sbpl_arm_env_(NULL), prm_(NULL), replan_params(20.0)
{
  rm_ = rm;
  cc_ = cc;
  as_ = as;
  df_ = df;
  planner_initialized_ = false;
  start_full.resize(7);
  goal_full.resize(7);
}

SBPLArmPlannerInterface::SBPLArmPlannerInterface(const ompl::base::SpaceInformationPtr &si, RobotModel *rm, CollisionChecker *cc, ActionSet* as, distance_field::PropagationDistanceField* df): 
  nh_("~"), planner_(NULL), sbpl_arm_env_(NULL), prm_(NULL), replan_params(20.0), si_(si)
{
  rm_ = rm;
  cc_ = cc;
  as_ = as;
  df_ = df;
  planner_initialized_ = false;
  start_full.resize(7);
  goal_full.resize(7);
  allocated_time_secs = 20.0;
  bforwardsearch = true;
  nh_.param("epsilon", epsilon_, 100);
  replan_params.initial_eps = epsilon_;
  replan_params.final_eps = epsilon_;
  replan_params.return_first_solution = false;

  pdef.reset(new ompl::base::ProblemDefinition(si));
  pathSimplifier = new ompl::geometric::PathSimplifier(si);
}

SBPLArmPlannerInterface::~SBPLArmPlannerInterface()
{
  if(planner_ != NULL)
    delete planner_;
  if(sbpl_arm_env_ != NULL)
    delete sbpl_arm_env_;
  if(prm_ != NULL)
    delete prm_;
}

bool SBPLArmPlannerInterface::init(std::string ns)
{
  if(!initializePlannerAndEnvironment(ns))
    return false;

  planner_initialized_ = true;
  ROS_INFO("The SBPL arm planner node initialized succesfully.");
  return true;
}

bool SBPLArmPlannerInterface::initializePlannerAndEnvironment(std::string ns)
{
  prm_ = new sbpl_arm_planner::PlanningParams();
  if(!prm_->init(ns))
    return false;

  grid_ = new sbpl_arm_planner::OccupancyGrid(df_);
  sbpl_arm_env_ = new sbpl_arm_planner::EnvironmentOMPLROBARM3D(si_, grid_, rm_, cc_, as_, prm_);

  if(!sbpl_arm_env_)
    return false;

  if(!as_->init(sbpl_arm_env_))
  {
    ROS_ERROR("Failed to initialize the action set.");
    return false;
  } 
  //as_->print();

  //initialize environment  
  planner_ = new PPMAPlanner(si_, sbpl_arm_env_, bforwardsearch, allocated_time_secs, &replan_params);
  //planner_ = new ARAPlanner(sbpl_arm_env_, true);
  //mha_planner_ = new MHAPlanner(sbpl_arm_env_, 2, true, true);

  //initialize arm planner environment
  if(!sbpl_arm_env_->initEnvironment())
  {
    ROS_ERROR("ERROR: initEnvironment failed");
    return false;
  }

  //initialize MDP 
  if(!sbpl_arm_env_->InitializeMDPCfg(&mdp_cfg_))
  {
    ROS_ERROR("ERROR: InitializeMDPCfg failed");
    return false;
  }

  //set epsilon
  //TODO: get from params
  planner_->set_initialsolution_eps(epsilon_);

  //set search mode (true - settle with first solution)
  planner_->set_search_mode(prm_->search_mode_);
  ROS_INFO("Initialized sbpl arm planning environment.");
  return true;
}

bool SBPLArmPlannerInterface::solve(const arm_navigation_msgs::PlanningSceneConstPtr& planning_scene,
                                    const arm_navigation_msgs::GetMotionPlan::Request &req,
                                    arm_navigation_msgs::GetMotionPlan::Response &res, std::vector<trajectory_msgs::JointTrajectoryPoint> &straj, vector<double> full_start, vector<double> full_goal, double &simplification_time) 
{
  start_full = full_start;
  //printf("SBPLStart: %f,%f,%f,%f,%f,%f,%f\n", start_full[0], start_full[1], start_full[2], start_full[3], start_full[4], start_full[5], start_full[6]);
  goal_full = full_goal;
  if(!planner_initialized_)
    return false;

  if(ppma)
  {
    ompl::base::StateSpacePtr space(planner_->getSpaceInformation()->getStateSpace());
    ompl::base::State *ompl_start = space->allocState();

    ompl_start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = start_full[0];
    ompl_start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = start_full[1];
    ompl_start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = start_full[2];
    ompl_start->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = start_full[3];
    ompl_start->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = start_full[4];
    ompl_start->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = start_full[5];
    ompl_start->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = start_full[6];

    if(planner_->getSpaceInformation()->isValid(ompl_start))
      ROS_INFO("[ompl] Start state is valid.");
    else
      ROS_ERROR("[ompl] Start state is NOT valid.");

    ompl::base::State *ompl_goal = space->allocState();

    ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = goal_full[0];
    ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = goal_full[1];
    ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = goal_full[2];
    ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = goal_full[3];
    ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = goal_full[4];
    ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = goal_full[5];
    ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = goal_full[6];

    if(planner_->getSpaceInformation()->isValid(ompl_goal))
      ROS_INFO("[ompl] Goal state is valid.");
    else
      {ROS_ERROR("[ompl] Goal state is NOT valid.");
       printf("The invalid goal state is given by %f,%f,%f,%f,%f,%f,%f\n", goal_full[0],goal_full[1],goal_full[2],goal_full[3],
                                                                            goal_full[4],goal_full[5],goal_full[6]);
       return false;
     }


    pdef->clearGoal();
    pdef->clearStartStates();
    pdef->clearSolutionPaths();
    pdef->setStartAndGoalStates(ompl_start,ompl_goal);
    pdef->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(si_)));

    sbpl_arm_env_->cost_obj = pdef->getOptimizationObjective();

    planner_->setProblemDefinition(pdef);
  }

  double distt;
  if(!cc_->isStateValid(goal_full, true, false, distt))
    ROS_INFO("before setting scene failed");

  // preprocess
  clock_t t_preprocess = clock();
  cc_->setPlanningScene(*planning_scene); 
  prm_->planning_frame_ = planning_scene->collision_map.header.frame_id;
  grid_->setReferenceFrame(prm_->planning_frame_);
  // TODO: set kinematics to planning frame
  double preprocess_time = (clock() - t_preprocess) / (double)CLOCKS_PER_SEC;

  if(!cc_->isStateValid(goal_full, true, false, distt))
    ROS_INFO("after setting scene failed");

  // plan
  clock_t t_plan = clock();
  res.robot_state = planning_scene->robot_state;
  if(!planToPosition(req,res, straj, simplification_time))
    return false;

  res_ = res;
  double plan_time = (clock() - t_plan) / (double)CLOCKS_PER_SEC;
  //ROS_INFO("t_plan: %0.3fsec  t_preprocess: %0.3fsec", plan_time, preprocess_time);

  return true;
}

bool SBPLArmPlannerInterface::setStart(const sensor_msgs::JointState &state)
{
  // std::vector<double> initial_positions;
  // if(!leatherman::getJointPositions(state, prm_->planning_joints_, initial_positions))
  // {
  //   ROS_ERROR("Start state does not contain the positions of the planning joints.");
  //   return false;
  // }
  //todo
  if(sbpl_arm_env_->setStartConfiguration(start_full) == 0)
  {
    ROS_ERROR("Environment failed to set start state. Not Planning.");
    return false;
  }
  if(planner_->set_start(mdp_cfg_.startstateid) == 0)///Changed here
  {
    ROS_ERROR("Failed to set start state. Not Planning.");
    return false;
  }
  ROS_INFO("start: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", start_full[0],start_full[1],start_full[2],start_full[3],start_full[4],start_full[5],start_full[6]);
  return true;
}

bool SBPLArmPlannerInterface::setGoal(vector<double> initial_positions)
{
  // std::vector<double> initial_positions;
  // if(!leatherman::getJointPositions(state, prm_->planning_joints_, initial_positions))
  // {
  //   ROS_ERROR("Goal state does not contain the positions of the planning joints.");
  //   return false;
  // }

  if(sbpl_arm_env_->setGoalConfiguration(initial_positions) == 0)
  {
    ROS_ERROR("Environment failed to set Goal state. Not Planning.");
    return false;
  }
  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)///changed here 
  {
    ROS_ERROR("Failed to set Goal state. Not Planning.");
    return false;
  }
  ROS_INFO("Goal: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", initial_positions[0],initial_positions[1],initial_positions[2],initial_positions[3],initial_positions[4],initial_positions[5],initial_positions[6]);
  return true;
}

bool SBPLArmPlannerInterface::setGoalPosition(const arm_navigation_msgs::Constraints &goals)
{
  std::vector<double> p(6,0);
  sbpl_arm_env_->getRobotModel()->computeFK(goal_full, sbpl_arm_env_->getRobotModel()->getPlanningLink(), p);

  geometry_msgs::Quaternion goalq;
  std::vector <std::vector <double> > sbpl_goal(1, std::vector<double> (11,0));  //Changed to include Quaternion
  std::vector <std::vector <double> > sbpl_tolerance(1, std::vector<double> (13,0));

  if(goals.position_constraints.size() != goals.orientation_constraints.size())
    ROS_WARN("There are %d position contraints and %d orientation constraints.", int(goals.position_constraints.size()),int(goals.orientation_constraints.size()));

  //currently only supports one goal
  sbpl_goal[0][0] = p[0];
  sbpl_goal[0][1] = p[1];
  sbpl_goal[0][2] = p[2];
  sbpl_goal[0][3] = p[3];
  sbpl_goal[0][4] = p[4];
  sbpl_goal[0][5] = p[5];

  //convert quaternion into roll,pitch,yaw //TODO
  // goalq = goals.orientation_constraints[0].orientation;
  // goalq.w += 0.001; //perturb quaternion if rpy will suffer from gimbal lock
  // leatherman::getRPY(goalq, sbpl_goal[0][3], sbpl_goal[0][4], sbpl_goal[0][5]);
 
  //6dof goal: 1, 3dof: 0, 7dof: 3  
  sbpl_goal[0][6] = 3;
 
  //orientation constraint as a quaternion 
  sbpl_goal[0][7] = goals.orientation_constraints[0].orientation.x;
  sbpl_goal[0][8] = goals.orientation_constraints[0].orientation.y;
  sbpl_goal[0][9] = goals.orientation_constraints[0].orientation.z;
  sbpl_goal[0][10] = goals.orientation_constraints[0].orientation.w;

  //allowable tolerance from goal
  if(goals.position_constraints[0].constraint_region_shape.dimensions.size() == 3)
  {
    sbpl_tolerance[0][0] = goals.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[1];
    sbpl_tolerance[0][2] = goals.position_constraints[0].constraint_region_shape.dimensions[2];
  }
  else
  {
    sbpl_tolerance[0][0] = goals.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[0];
  }
  sbpl_tolerance[0][3] = goals.orientation_constraints[0].absolute_roll_tolerance;
  sbpl_tolerance[0][4] = goals.orientation_constraints[0].absolute_pitch_tolerance;
  sbpl_tolerance[0][5] = goals.orientation_constraints[0].absolute_yaw_tolerance;

  //7-DOF Goal tolerance
  //TODO(Karthik) : Read tolerance from config file  
  sbpl_tolerance[0][6] = 0.05;
  sbpl_tolerance[0][7] = 0.05;
  sbpl_tolerance[0][8] = 0.05;
  sbpl_tolerance[0][9] = 0.05;
  sbpl_tolerance[0][10] = 0.05;
  sbpl_tolerance[0][11] = 0.05;
  sbpl_tolerance[0][12] = 0.05;

  ROS_INFO("goal xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)  (quat: %0.3f %0.3f %0.3f %0.3f)", prm_->planning_frame_.c_str(),sbpl_goal[0][0],sbpl_goal[0][1],sbpl_goal[0][2],sbpl_tolerance[0][0],sbpl_goal[0][3],sbpl_goal[0][4],sbpl_goal[0][5], sbpl_tolerance[0][1], goals.orientation_constraints[0].orientation.x, goals.orientation_constraints[0].orientation.y, goals.orientation_constraints[0].orientation.z, goals.orientation_constraints[0].orientation.w);

  //set sbpl environment goal
  if(!sbpl_arm_env_->setGoalPosition(sbpl_goal, sbpl_tolerance))
  {
    ROS_ERROR("Failed to set goal state. Perhaps goal position is out of reach. Exiting.");
    return false;
  }

  //set planner goal	
  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)///Changed here
  {
    ROS_ERROR("Failed to set goal state. Exiting.");
    return false;
  }

  return true;
}

bool SBPLArmPlannerInterface::setStartPosition(const arm_navigation_msgs::Constraints &starts, vector<double> start)
{
  std::vector<double> p(6,0);
  sbpl_arm_env_->getRobotModel()->computeFK(start_full, sbpl_arm_env_->getRobotModel()->getPlanningLink(), p);

  geometry_msgs::Quaternion startq;
  std::vector <std::vector <double> > sbpl_start(1, std::vector<double> (11,0));  //Changed to include Quaternion
  std::vector <std::vector <double> > sbpl_tolerance(1, std::vector<double> (12,0));

  if(starts.position_constraints.size() != starts.orientation_constraints.size())
    ROS_WARN("There are %d position contraints and %d orientation constraints.", int(starts.position_constraints.size()),int(starts.orientation_constraints.size()));

  //currently only supports one start
  sbpl_start[0][0] = p[0];
  sbpl_start[0][1] = p[1];
  sbpl_start[0][2] = p[2];
  sbpl_start[0][3] = p[3];
  sbpl_start[0][4] = p[4];
  sbpl_start[0][5] = p[5];

//fahad DONT NEED IT
  // startq = starts.orientation_constraints[0].orientation;       
  // startq.w += 0.001; //perturb quaternion if rpy will suffer from gimbal lock
  // leatherman::getRPY(startq, sbpl_start[0][3], sbpl_start[0][4], sbpl_start[0][5]);
 
  //6dof start: true, 3dof: false 
  sbpl_start[0][6] = true;
 
  //orientation constraint as a quaternion 
  //fahad TODOOOOO
  sbpl_start[0][7] = starts.orientation_constraints[0].orientation.x;
  sbpl_start[0][8] = starts.orientation_constraints[0].orientation.y;
  sbpl_start[0][9] = starts.orientation_constraints[0].orientation.z;
  sbpl_start[0][10] = starts.orientation_constraints[0].orientation.w;

  //allowable tolerance from start
  if(starts.position_constraints[0].constraint_region_shape.dimensions.size() == 3)
  {
    sbpl_tolerance[0][0] = starts.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = starts.position_constraints[0].constraint_region_shape.dimensions[1];
    sbpl_tolerance[0][2] = starts.position_constraints[0].constraint_region_shape.dimensions[2];
  }
  else
  {
    sbpl_tolerance[0][0] = starts.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = starts.position_constraints[0].constraint_region_shape.dimensions[0];
    sbpl_tolerance[0][1] = starts.position_constraints[0].constraint_region_shape.dimensions[0];
  }
  sbpl_tolerance[0][3] = starts.orientation_constraints[0].absolute_roll_tolerance;
  sbpl_tolerance[0][4] = starts.orientation_constraints[0].absolute_pitch_tolerance;
  sbpl_tolerance[0][5] = starts.orientation_constraints[0].absolute_yaw_tolerance;

  ROS_INFO("start xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)  (quat: %0.3f %0.3f %0.3f %0.3f)", prm_->planning_frame_.c_str(),sbpl_start[0][0],sbpl_start[0][1],sbpl_start[0][2],sbpl_tolerance[0][0],sbpl_start[0][3],sbpl_start[0][4],sbpl_start[0][5], sbpl_tolerance[0][1], starts.orientation_constraints[0].orientation.x, starts.orientation_constraints[0].orientation.y, starts.orientation_constraints[0].orientation.z, starts.orientation_constraints[0].orientation.w);

  //set sbpl environment start
  if(!sbpl_arm_env_->setStartPosition(sbpl_start, sbpl_tolerance))
  {
    ROS_ERROR("Failed to set start state. Perhaps start position is out of reach. Exiting.");
    return false;
  }

  //set planner start  
  if(planner_->set_start(mdp_cfg_.startstateid) == 0)///Changed here
  {
    ROS_ERROR("Failed to set start state. Exiting.");
    return false;
  }

  return true;
}

bool SBPLArmPlannerInterface::planKinematicPath(const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
{
  if(!planner_initialized_)
  {
    ROS_ERROR("Hold up a second...the planner isn't initialized yet. Try again in a second or two.");
    return false;
  }

  if(req.motion_plan_request.goal_constraints.position_constraints.empty())
  {
    ROS_ERROR("There aren't any goal pose constraints in the request message. We need those to plan :).");
    return false;
  }

  std::vector<trajectory_msgs::JointTrajectoryPoint> straj;
  double stime = 0;
  if(!planToPosition(req, res, straj, stime))
    return false;

  return true;
}

bool SBPLArmPlannerInterface::plan(trajectory_msgs::JointTrajectory &traj, std::vector<trajectory_msgs::JointTrajectoryPoint> &straj, double &totalTime, double &simplification_time)
{
  static bool first = true;
  bool b_ret = false;
  std::vector<int> solution_state_ids;

  //reinitialize the search space
  planner_->force_planning_from_scratch();

  //plan
  
    // ReplanParams replan_params(100);///Changed here
    // double initialEpsilon = 100.0;
    // //replan_params.inflation_eps = initialEpsilon;
    // //replan_params.anchor_eps = 1.0;
    // replan_params.max_time = prm_->allowed_time_;
    // //replan_params.use_anchor = true;
    // replan_params.initial_eps = initialEpsilon;
    // replan_params.return_first_solution = false;
    // replan_params.final_eps = initialEpsilon;
    // replan_params.bds_type = mha_planner::BDSType::F2F;
    // replan_params.mha_type = mha_planner::MHAType::FOCAL;
    int solution_cost_;
    //double totalTime;
    b_ret = planner_->replan(&solution_state_ids, replan_params, &solution_cost_, totalTime);//Changed here
    //b_ret = planner_->replan(&solution_state_ids, replan_params, &solution_cost_);
    // getchar();

    if(ppma)
    {
      if(b_ret)
      {
        //printf("The planner found a solution it seems like\n");
        traj.header.frame_id = prm_->planning_frame_;
        traj.joint_names = prm_->planning_joints_;

        ret_path.reset(new ompl::geometric::PathGeometric(si_));
        ret_path2.reset(new ompl::geometric::PathGeometric(si_));
        ret_path_g.reset(new ompl::geometric::PathGeometric(si_));
        ret_path = planner_->getProblemDefinition()->getSolutionPath();
        ret_path2 = planner_->getProblemDefinition()->getSolutionPath();

        ret_path_g = boost::dynamic_pointer_cast<ompl::geometric::PathGeometric>(ret_path); 
        traj.points.resize(ret_path_g->getStateCount());

        //path simplification
        ompl::geometric::PathGeometric ret_path_g2 = dynamic_cast<ompl::geometric::PathGeometric&>(*ret_path2);

        chrono::time_point<chrono::monotonic_clock> start, end;
        start = chrono::monotonic_clock::now();
        bool b1 = pathSimplifier->reduceVertices(ret_path_g2);
        bool b2 = pathSimplifier->collapseCloseVertices(ret_path_g2);
        end = chrono::monotonic_clock::now();
        chrono::duration<double> elapsed_seconds = end-start;

        ROS_INFO("[sbpl] reduce: %d  collapse:%d", b1, b2);
   
        simplification_time = elapsed_seconds.count();

        for(size_t i = 0; i < ret_path_g->getStateCount(); i++){

          const ompl::base::RealVectorStateSpace::StateType* path_state = (ret_path_g->getState(i))->as<ompl::base::RealVectorStateSpace::StateType>();
          std::vector<double> state_angles(prm_->num_joints_,0);
          for(int j = 0; j < prm_->num_joints_; j++)
          {
            state_angles[j] = path_state->values[j];
          }

          traj.points[i].positions.resize(prm_->num_joints_);
          for (int p = 0; p < prm_->num_joints_; ++p){
            traj.points[i].positions[p] = angles::normalize_angle(state_angles[p]); 
          }
        }

        straj.resize(ret_path_g2.getStateCount());
        for(size_t i = 0; i < ret_path_g2.getStateCount(); i++){

          const ompl::base::RealVectorStateSpace::StateType* path_state = (ret_path_g2.getState(i))->as<ompl::base::RealVectorStateSpace::StateType>();
          std::vector<double> state_angles(prm_->num_joints_,0);
          for(int j = 0; j < prm_->num_joints_; j++)
          {
            state_angles[j] = path_state->values[j];
          }

          straj[i].positions.resize(prm_->num_joints_);
          for (int p = 0; p < prm_->num_joints_; ++p){
            straj[i].positions[p] = angles::normalize_angle(state_angles[p]); 
          }
        }


      }
      else{
        traj.points.clear();
      }
      planner_->getProblemDefinition()->clearSolutionPaths();
      planner_->clear();
      return b_ret;
    }

}

bool SBPLArmPlannerInterface::planToPosition(const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res, std::vector<trajectory_msgs::JointTrajectoryPoint> &straj, double &simplification_time)
{
  starttime = clock();
  int status = 0;
  prm_->allowed_time_ = req.motion_plan_request.allowed_planning_time.toSec();
  req_ = req.motion_plan_request;

  if(!canServiceRequest(req))
    return false;

  //TODO: transform goal pose into reference_frame
  arm_navigation_msgs::Constraints goal_constraints = req.motion_plan_request.goal_constraints;

  // find IK here
  /*
  geometry_msgs::Pose gpose, gpose_out;
  gpose.position = req.motion_plan_request.goal_constraints.position_constraints[0].position;
  gpose.orientation = req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation;
  sbpl_arm_planner::transformPose(pscene_, gpose, gpose_out, req.motion_plan_request.goal_constraints[0].position_constraints[0].header.frame_id, prm_->planning_frame);
  goal_constraints.orientation_constraints[0].orientation = gpose_out.orientation;
  */

  // set start
  ROS_DEBUG("Setting start.");
  if(!setStart(req.motion_plan_request.start_state.joint_state))
  {
    status = -1;
    ROS_ERROR("Failed to set initial configuration of robot.");
  }

  // set goal
  ROS_DEBUG("Setting goal.");
  if(!setGoalPosition(goal_constraints) && status == 0)
  {
    status = -2;
    ROS_ERROR("Failed to set goal position.");
  }

////////////////////////////////////////////////////////////fahad
  ROS_DEBUG("Setting goal backward.");
  // std::vector<double> goal = sbpl_arm_env_->getGoal();
  // std::vector<double> goal_full(7), goal_seed(7,1.0);
  // double dist=0;
  // // do{
  //   sbpl_arm_env_->getRobotModel()->computeIK(goal, goal_seed, goal_full);
     
  // } while (!cc_->isStateValid(goal_full, true, false, dist) );
  

  if(!setGoal(goal_full))    
  {
    status = -1;
    ROS_ERROR("Failed to set goal configuration of robot.");
  }

  ROS_DEBUG("Setting start backward.");
  std::vector<double> initial_positions;  //dummy
  // if(!leatherman::getJointPositions(req.motion_plan_request.start_state.joint_state, prm_->planning_joints_, initial_positions))
  // {
  //   ROS_ERROR("Start state does not contain the positions of the planning joints.");
  //   return false;
  // }
  if(!setStartPosition(goal_constraints, initial_positions) && status == 0)
  {
    status = -2;
    ROS_ERROR("Failed to set goal position.");
  }
///////// //////////////////////////////////////////
  
  // sbpl_arm_env_->GenerateRandomState();
  // return false;
  // plan 
  double totalTime = 0;

  ROS_DEBUG("Calling planner"); 
  if(status == 0 && plan(res.trajectory.joint_trajectory, straj, totalTime, simplification_time))
  {
    res.trajectory.joint_trajectory.header.seq = req.motion_plan_request.goal_constraints.position_constraints[0].header.seq; 
    res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    // fill in the waypoint times (not very intelligently)
    res.trajectory.joint_trajectory.points[0].time_from_start.fromSec(prm_->waypoint_time_);
    for(size_t i = 1; i < res.trajectory.joint_trajectory.points.size(); i++)
      res.trajectory.joint_trajectory.points[i].time_from_start.fromSec(res.trajectory.joint_trajectory.points[i-1].time_from_start.toSec() + prm_->waypoint_time_);

    //res.planning_time = ros::Duration((clock() - starttime) / (double)CLOCKS_PER_SEC);//ros::Duration(totalTime);//
    res.planning_time = ros::Duration(totalTime);
    
    // shortcut path
    if(prm_->shortcut_path_)
    {
      trajectory_msgs::JointTrajectory straj;
      if(!interpolateTrajectory(cc_, res.trajectory.joint_trajectory.points, straj.points))
        ROS_WARN("Failed to interpolate planned trajectory with %d waypoints before shortcutting.", int(res.trajectory.joint_trajectory.points.size()));
      
      shortcutTrajectory(cc_, straj.points,res.trajectory.joint_trajectory.points);
    }

    // interpolate path
    if(prm_->interpolate_path_)
    {
      trajectory_msgs::JointTrajectory itraj = res.trajectory.joint_trajectory;
      interpolateTrajectory(cc_, itraj.points, res.trajectory.joint_trajectory.points);
    }

    if(prm_->print_path_)
      leatherman::printJointTrajectory(res.trajectory.joint_trajectory, "path");
  }
  else
  {
    status = -3;
    res.planning_time = ros::Duration(totalTime);
    ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds).", prm_->allowed_time_);
  }

  if(status == 0)
    return true;

  return false;
}

bool SBPLArmPlannerInterface::canServiceRequest(const arm_navigation_msgs::GetMotionPlan::Request &req)
{
  // check for an empty start state
  if(req.motion_plan_request.start_state.joint_state.position.empty())
  {
    ROS_ERROR("No start state given. Unable to plan.");
    return false;
  }

  // check if position & orientation constraints is empty
  if(req.motion_plan_request.goal_constraints.position_constraints.empty() || 
      req.motion_plan_request.goal_constraints.orientation_constraints.empty())
  {
    ROS_ERROR("Position or orientation constraint is empty. Expecting a 6D end effector pose constraint. Exiting.");
    return false;
  }
  
  // check if there is more than one goal constraint
  if(req.motion_plan_request.goal_constraints.position_constraints.size() > 1 || 
      req.motion_plan_request.goal_constraints.orientation_constraints.size() > 1)
    ROS_WARN("The planning request message contains %d position and %d orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", int(req.motion_plan_request.goal_constraints.position_constraints.size()), int(req.motion_plan_request.goal_constraints.orientation_constraints.size()));

  return true;
}

std::map<std::string, double>  SBPLArmPlannerInterface::getPlannerStats()
{
  std::map<std::string, double> stats;
  stats["initial solution planning time"] = planner_->get_initial_eps_planning_time();
  stats["initial epsilon"] = planner_->get_initial_eps();
  stats["initial solution expansions"] = planner_->get_n_expands_init_solution();
  stats["final epsilon planning time"] = planner_->get_final_eps_planning_time();
  stats["final epsilon"] = planner_->get_final_epsilon();
  stats["solution epsilon"] = planner_->get_solution_eps();
  stats["expansions"] = planner_->get_n_expands();
  stats["solution cost"] = solution_cost_;
  return stats;
}

visualization_msgs::MarkerArray SBPLArmPlannerInterface::getCollisionModelTrajectoryMarker()
{
  visualization_msgs::MarkerArray ma, ma1;
  std::vector<std::vector<double> > traj;

  if(res_.trajectory.joint_trajectory.points.empty())
  {
    ROS_ERROR("No trajectory found to visualize yet. Plan a path first.");
    return ma;
  }

  traj.resize(res_.trajectory.joint_trajectory.points.size());
  double cinc = 1.0/double(res_.trajectory.joint_trajectory.points.size());
  for(size_t i = 0; i < res_.trajectory.joint_trajectory.points.size(); ++i)
  {
    traj[i].resize(res_.trajectory.joint_trajectory.points[i].positions.size());
    for(size_t j = 0; j < res_.trajectory.joint_trajectory.points[i].positions.size(); j++)
      traj[i][j] = res_.trajectory.joint_trajectory.points[i].positions[j];

    ma1 = cc_->getCollisionModelVisualization(traj[i]);

    for(size_t j = 0; j < ma1.markers.size(); ++j)
    {
      ma1.markers[j].color.r = 0.1;
      ma1.markers[j].color.g = cinc*double(res_.trajectory.joint_trajectory.points.size()-(i+1));
      ma1.markers[j].color.b = cinc*double(i);
    }
    ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
  }

  for(size_t i = 0; i < ma.markers.size(); ++i)
  {
    ma.markers[i].ns = "trajectory";
    ma.markers[i].id = i;
  }

  return ma;
}

visualization_msgs::MarkerArray SBPLArmPlannerInterface::getVisualization(std::string type)
{
  visualization_msgs::MarkerArray ma;
  if(type.compare("goal") == 0)
  {
    if(req_.goal_constraints.position_constraints.empty())
    {
      ROS_WARN("Failed to get visualization marker for goals because no position constraints found.");
      return visualization_msgs::MarkerArray();
    }

    std::vector<std::vector<double> > poses(req_.goal_constraints.position_constraints.size(),std::vector<double>(6,0));
    for(size_t i = 0; i < req_.goal_constraints.position_constraints.size(); ++i)
    {
      poses[i][0] = req_.goal_constraints.position_constraints[i].position.x;
      poses[i][1] = req_.goal_constraints.position_constraints[i].position.y;
      poses[i][2] = req_.goal_constraints.position_constraints[i].position.z;

      if(req_.goal_constraints.orientation_constraints.size() > i)
        leatherman::getRPY(req_.goal_constraints.orientation_constraints[i].orientation, poses[i][3], poses[i][4], poses[i][5]);
      else
      {
        poses[i][3] = 0;
        poses[i][4] = 0;
        poses[i][5] = 0;
      }
    }
    ma = viz::getPosesMarkerArray(poses, req_.goal_constraints.position_constraints[0].header.frame_id, "goals", 0);
  }
  else if(type.compare("expansions") == 0)
  {
    std::vector<std::vector<double> > expanded_states;
    sbpl_arm_env_->getExpandedStates(&(expanded_states));
    if(!expanded_states.empty())
    {
      std::vector<std::vector<double> > colors(2, std::vector<double>(4,0));
      colors[0][0] = 1;
      colors[0][3] = 1;
      colors[1][1] = 1;
      colors[1][3] = 1;
      ma = viz::getCubesMarkerArray(expanded_states, 0.01, colors, prm_->planning_frame_, "expansions", 0);
    }
  }
  else
    ma = sbpl_arm_env_->getVisualization(type);

  return ma;
}

void SBPLArmPlannerInterface::packageMHAStats(vector<string> &stat_names,
vector<double> &stats,
int solution_cost,
size_t solution_size) {
  stat_names.resize(10);
  stats.resize(10);
  stat_names[0] = "total plan time";
  stat_names[1] = "initial solution planning time";
  stat_names[2] = "epsilon 1";
  stat_names[3] = "initial solution expansions";
  stat_names[4] = "final epsilon planning time";
  stat_names[5] = "epsilon 2";
  stat_names[6] = "solution epsilon";
  stat_names[7] = "expansions";
  stat_names[8] = "solution cost";
  stat_names[9] = "path length";
  vector<PlannerStats> planner_stats;
  planner_->get_search_stats(&planner_stats);//Changed here
  if (planner_stats.empty()) {
    stats[0] = -1;
    stats[1] = -1;
    stats[2] = -1;
    stats[3] = -1;
    stats[4] = -1;
    stats[5] = -1;
    stats[6] = -1;
    stats[7] = -1;
    stats[8] = -1;
    stats[9] = -1;
  } else {
  // Take stats only for the first solution, since this is not anytime currently
    stats[0] = planner_stats[0].time;
    stats[1] = stats[0];
    stats[2] = 3.00;          //TODOOOOOOOO
    stats[3] = planner_stats[0].expands;
    stats[4] = stats[0];
    stats[5] = stats[2];
    stats[6] = stats[2];
    stats[7] = stats[3];
    stats[8] = static_cast<double>(planner_stats[0].cost);
    stats[9] = static_cast<double>(solution_size);
  }

}

void SBPLArmPlannerInterface::visualizeOMPLpath(const trajectory_msgs::JointTrajectory &traj)
{
  ompl::base::StateSpacePtr space(planner_->getSpaceInformation()->getStateSpace());
  ompl::base::State *parent = space->allocState();
  ompl::base::State *child = space->allocState();

  for(size_t i = 0; i < traj.points.size(); ++i)
  {
    for(size_t j = 0; j < 7; ++j)
      parent->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = traj.points[i].positions[j];
    if(i != traj.points.size() - 1)
      for(size_t j = 0; j < 7; ++j)
        child->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = traj.points[i+1].positions[j]; 
    else
      for(size_t j = 0; j < 7; ++j)
        child->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = traj.points[i].positions[j];

    double edge_length = si_->distance(parent, child);
    int num_segments = static_cast<int>(edge_length / 0.0348);
    double interp_factor = 1.0 / static_cast<double>(num_segments);
    bool valid = true;
    ompl::base::State *parstate = parent;
    ompl::base::State *interp_state = si_->allocState();
    for (double t = 0 ; t <=1; t+=interp_factor) {
       si_->getStateSpace()->interpolate(parent, child, t, interp_state);
       sbpl_arm_env_->VisualizeContState(interp_state,parstate, false, false);
       parstate = interp_state;
    }
    si_->freeState(interp_state);

  }
} 
