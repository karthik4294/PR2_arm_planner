/*
 * Copyright (c) 2010, Maxim Likhachev
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
/** \author Benjamin Cohen */

#include <sbpl_arm_planner/environment_omplrobarm3d.h>
//#include <bfs3d/BFS_Util.hpp>
#include <leatherman/viz.h>

#define DEG2RAD(d) ((d)*(M_PI/180.0))
#define RAD2DEG(r) ((r)*(180.0/M_PI))

using namespace std;

bool use_visualization_ = false;
bool slow_visualize = false;

namespace sbpl_arm_planner
{

EnvironmentOMPLROBARM3D::EnvironmentOMPLROBARM3D(OccupancyGrid *grid, RobotModel *rmodel, CollisionChecker *cc, ActionSet* as, PlanningParams *pm) : bfs_(NULL), bfs_b(NULL)
{
  grid_ = grid;
  rmodel_ = rmodel;
  cc_ = cc;
  as_ = as;
  prm_ = pm;
  getHeuristic_ = &sbpl_arm_planner::EnvironmentOMPLROBARM3D::getXYZHeuristic;
  getHeuristicBack_ = &sbpl_arm_planner::EnvironmentOMPLROBARM3D::getXYZHeuristicBack;
  pviz_.setReferenceFrame("base_footprint");
  tree_pub = nh.advertise<visualization_msgs::Marker>("tree_marker", 1000);
  bfs_pub = nh.advertise<visualization_msgs::MarkerArray>("bfs_marker", 1000);
}

EnvironmentOMPLROBARM3D::EnvironmentOMPLROBARM3D(const ompl::base::SpaceInformationPtr &si, OccupancyGrid *grid, RobotModel *rmodel, CollisionChecker *cc, ActionSet* as, PlanningParams *pm) : bfs_(NULL), bfs_b(NULL), si_(si)
{
  grid_ = grid;
  rmodel_ = rmodel;
  cc_ = cc;
  as_ = as;
  prm_ = pm;
  getHeuristic_ = &sbpl_arm_planner::EnvironmentOMPLROBARM3D::getXYZHeuristic;
  getHeuristicBack_ = &sbpl_arm_planner::EnvironmentOMPLROBARM3D::getXYZHeuristicBack;
  pviz_.setReferenceFrame("base_footprint");
  cont_hash_counter = SBPL_OMPL_MAXSTATESFORLOOKUP;
  tree_pub = nh.advertise<visualization_msgs::Marker>("tree_marker", 1000);
  bfs_pub = nh.advertise<visualization_msgs::MarkerArray>("bfs_marker", 1000);
}

EnvironmentOMPLROBARM3D::~EnvironmentOMPLROBARM3D()
{
  if(bfs_ != NULL)
    delete bfs_;

  if(bfs_b != NULL)
    delete bfs_b;

  for(size_t i = 0; i < pdata_.StateID2CoordTable.size(); i++)
  {
    delete pdata_.StateID2CoordTable.at(i);
    pdata_.StateID2CoordTable.at(i) = NULL;
  }
  pdata_.StateID2CoordTable.clear();

  if(pdata_.Coord2StateIDHashTable != NULL)
  {
    delete [] pdata_.Coord2StateIDHashTable;
    pdata_.Coord2StateIDHashTable = NULL;
  }
}

bool EnvironmentOMPLROBARM3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  MDPCfg->goalstateid = pdata_.goal_entry->stateID;
  MDPCfg->startstateid = pdata_.start_entry->stateID;
  return true;
}

int EnvironmentOMPLROBARM3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  return (*this.*getHeuristic_)(FromStateID,ToStateID);
}

int EnvironmentOMPLROBARM3D::GetFromToHeuristicBack(int FromStateID, int ToStateID)
{
  return (*this.*getHeuristicBack_)(FromStateID,ToStateID);
}

int EnvironmentOMPLROBARM3D::GetGoalHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal");
    return -1;
  }
#endif
  return GetFromToHeuristic(stateID, pdata_.goal_entry->stateID);
  
  // EnvOMPLROBARM3DHashEntry_t* HashEntry1 = pdata_.StateID2CoordTable[stateID];
  // EnvOMPLROBARM3DHashEntry_t* HashEntry2 = pdata_.StateID2CoordTable[pdata_.goal_entry->stateID];

  // std::vector<double> from_config(prm_->num_joints_,0);
  // coordToAngles(HashEntry1->coord, from_config);

  // std::vector<double> to_config(prm_->num_joints_,0);
  // coordToAngles(HashEntry2->coord, to_config);

  // ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(7));

  // const ompl::base::State *from_config_state = space->allocState();
  // const ompl::base::State *to_config_state = space->allocState();

  // for(int i = 0; i < prm_->num_joints_; i++)
  // {
  //   from_config_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = from_config[i];
  // }

  // for(int i = 0; i < prm_->num_joints_; i++)
  // {
  //   to_config_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = to_config[i];
  // }

  // return GetContEdgeCost(from_config_state, to_config_state);

}

int EnvironmentOMPLROBARM3D::GetGoalHeuristic(int q_id, int stateID, int stateID_front)
{
if (q_id)
  state_id_last = stateID_front;
  // state_id_last = pdata_.goal_entry->stateID;
#if DEBUG_HEUR
  if(stateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal");
    return -1;
  }
#endif
  switch(q_id){
    case 0:         //3D to goal
      return GetFromToHeuristic(stateID, pdata_.goal_entry->stateID); 
    case 1:         //Joint Euc between states
    {
      EnvOMPLROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[stateID];
      EnvOMPLROBARM3DHashEntry_t* FromHashEntry_f = pdata_.StateID2CoordTable[stateID_front];
      double x, y, z, x_f, y_f, z_f;

      std::vector<int> scoord(prm_->num_joints_,0), scoord_f(prm_->num_joints_,0);
      std::vector<double> angles(prm_->num_joints_,0), angles_f(prm_->num_joints_,0);
      for(int i = 0; i < prm_->num_joints_; i++)
      {
        scoord[i] = FromHashEntry->coord.at(i);
        scoord_f[i] = FromHashEntry_f->coord.at(i);
      }
      coordToAngles(scoord, angles);
      coordToAngles(scoord_f, angles_f);
      return 10000 * sqrt(computeMinUnsignedAngleDiff(angles[0], angles_f[0])*computeMinUnsignedAngleDiff(angles[0], angles_f[0]) + computeMinUnsignedAngleDiff(angles[1], angles_f[1])*computeMinUnsignedAngleDiff(angles[1], angles_f[1]) + computeMinUnsignedAngleDiff(angles[2], angles_f[2])*computeMinUnsignedAngleDiff(angles[2], angles_f[2])
                        +computeMinUnsignedAngleDiff(angles[3] , angles_f[3])*computeMinUnsignedAngleDiff(angles[3], angles_f[3]) + computeMinUnsignedAngleDiff(angles[4], angles_f[4])*computeMinUnsignedAngleDiff(angles[4], angles_f[4]) + computeMinUnsignedAngleDiff(angles[5], angles_f[5])*computeMinUnsignedAngleDiff(angles[6], angles_f[6])
                        +computeMinUnsignedAngleDiff(angles[6] , angles_f[6])*computeMinUnsignedAngleDiff(angles[6], angles_f[6]));
    //used for interpolated collision 
    } 
    case 2:     //Task Euc between states
    {
      EnvOMPLROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[stateID];
      EnvOMPLROBARM3DHashEntry_t* FromHashEntry_f = pdata_.StateID2CoordTable[stateID_front];
      double x, y, z, x_f, y_f, z_f;    
      grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x, y, z);
      grid_->gridToWorld(FromHashEntry_f->xyz[0],FromHashEntry_f->xyz[1],FromHashEntry_f->xyz[2], x_f, y_f, z_f);
      printf("dist %f\n", getEuclideanDistance(x, y, z, x_f, y_f, z_f) * prm_->cost_per_meter_);
      return (int)1000 * getEuclideanDistance(x, y, z, x_f, y_f, z_f) * prm_->cost_per_meter_;
    }        
    case 3:       //3D between states
    {
        return abs(GetFromToHeuristic(stateID, pdata_.goal_entry->stateID) - GetFromToHeuristic(stateID_front, pdata_.goal_entry->stateID)); 
    }
  }
}

int EnvironmentOMPLROBARM3D::GetStartHeuristic(int stateID)
{
#if DEBUG_HEUR
  if(stateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal\n");
    return -1;
  }
#endif

  return GetFromToHeuristicBack(stateID, pdata_.start_entry->stateID);
}

int EnvironmentOMPLROBARM3D::GetStartHeuristic(int q_id, int stateID, int stateID_front)
{
if (q_id)
  state_id_last_b = stateID_front;
  // state_id_last_b = pdata_.start_entry->stateID;
#if DEBUG_HEUR
  if(stateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal");
    return -1;
  }
#endif
  switch(q_id){
    case 0:         //3D to goal
      return GetFromToHeuristicBack(stateID, pdata_.start_entry->stateID); 
    case 1:         //Joint Euc between states
    {
      EnvOMPLROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[stateID];
      EnvOMPLROBARM3DHashEntry_t* FromHashEntry_f = pdata_.StateID2CoordTable[stateID_front];
      double x, y, z, x_f, y_f, z_f;

      std::vector<int> scoord(prm_->num_joints_,0), scoord_f(prm_->num_joints_,0);
      std::vector<double> angles(prm_->num_joints_,0), angles_f(prm_->num_joints_,0);
      for(int i = 0; i < prm_->num_joints_; i++)
      {
        scoord[i] = FromHashEntry->coord.at(i);
        scoord_f[i] = FromHashEntry_f->coord.at(i);
      }
      coordToAngles(scoord, angles);
      coordToAngles(scoord_f, angles_f);
      return 10000 * sqrt(computeMinUnsignedAngleDiff(angles[0], angles_f[0])*computeMinUnsignedAngleDiff(angles[0], angles_f[0]) + computeMinUnsignedAngleDiff(angles[1], angles_f[1])*computeMinUnsignedAngleDiff(angles[1], angles_f[1]) + computeMinUnsignedAngleDiff(angles[2], angles_f[2])*computeMinUnsignedAngleDiff(angles[2], angles_f[2])
                        +computeMinUnsignedAngleDiff(angles[3] , angles_f[3])*computeMinUnsignedAngleDiff(angles[3], angles_f[3]) + computeMinUnsignedAngleDiff(angles[4], angles_f[4])*computeMinUnsignedAngleDiff(angles[4], angles_f[4]) + computeMinUnsignedAngleDiff(angles[5], angles_f[5])*computeMinUnsignedAngleDiff(angles[6], angles_f[6])
                        +computeMinUnsignedAngleDiff(angles[6] , angles_f[6])*computeMinUnsignedAngleDiff(angles[6], angles_f[6]));
    //used for interpolated collision 
    } 
    case 2:     //Task Euc between states
    {
      EnvOMPLROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[stateID];
      EnvOMPLROBARM3DHashEntry_t* FromHashEntry_f = pdata_.StateID2CoordTable[stateID_front];
      double x, y, z, x_f, y_f, z_f;    
      grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x, y, z);
      grid_->gridToWorld(FromHashEntry_f->xyz[0],FromHashEntry_f->xyz[1],FromHashEntry_f->xyz[2], x_f, y_f, z_f);
      // printf("dist %f\n", getEuclideanDistance(x, y, z, x_f, y_f, z_f) * prm_->cost_per_meter_);
      return (int)1000 * getEuclideanDistance(x, y, z, x_f, y_f, z_f) * prm_->cost_per_meter_;
    }        
    case 3:       //3D between states
    {
        return abs(GetFromToHeuristicBack(stateID, pdata_.start_entry->stateID) - GetFromToHeuristicBack(stateID_front, pdata_.start_entry->stateID)); 
    }
  }
}

void EnvironmentOMPLROBARM3D::GetFrontJointState(vector<double> &angles, bool forward)
{
  EnvOMPLROBARM3DHashEntry_t* FromHashEntry;
  if (forward)
    FromHashEntry = pdata_.StateID2CoordTable[state_id_last];
  else
    FromHashEntry = pdata_.StateID2CoordTable[state_id_last_b];
  std::vector<int> scoord(prm_->num_joints_,0);
  for(int i = 0; i < prm_->num_joints_; i++)
  {
    scoord[i] = FromHashEntry->coord.at(i);
  }
  coordToAngles(scoord, angles);
}

int EnvironmentOMPLROBARM3D::SizeofCreatedEnv()
{
  return pdata_.StateID2CoordTable.size();
}

void EnvironmentOMPLROBARM3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG_HEUR
  if(stateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvOMPLROBARM3DHashEntry_t* HashEntry = pdata_.StateID2CoordTable[stateID];

  bool bGoal = false;
  if(stateID == pdata_.goal_entry->stateID)
    bGoal = true;

  if(stateID == pdata_.goal_entry->stateID && bVerbose)
  {
    //ROS_DEBUG_NAMED(fOut, "the state is a goal state\n");
    bGoal = true;
  }

  printJointArray(fOut, HashEntry, bGoal, bVerbose);
}

void EnvironmentOMPLROBARM3D::PrintEnv_Config(FILE* fOut)
{
  ROS_ERROR("ERROR in pdata_... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentOMPLROBARM3D::GenerateRandomState()
{
  // 0.5 -0.5 0.75
  BodyPose body;
  body.x = 0;
  body.y = 0;
  body.z = 0.100000;
  vector<double> left_arm{ 1.500,  1.000,  0.006, -1.400,  0.001, -0.744,  0.000};
  vector<double> pose(6,0);

  FILE *fout = fopen("/home/fahad/catkin_ws/src/single_arm/single_arm_benchmarks/benchmark_manipulation_tests/armplanning_tests.yaml", "w");
  fprintf(fout, "experiments:\n\n");
  for (int i = 0; i < 150; i++)
  {
    printf("%d\n", i);
      
    vector<double> start(7);
    vector<double> goal(7);
    vector<double> seed_start{-0.506396, -0.132026, -1.453629, -1.834122, -1.727646, -1.618609, -0.255917};
    vector<double> seed_goal{-0.506396, -0.132026, -1.453629, -1.834122, -1.727646, -1.618609, -0.255917};
    // 
    double dist=0;

    int test_num = i;

    fprintf(fout, "  - test: test_%d\n", test_num);
//////start
    vector<double> pose{0.456899, -0.117160, 0.979574, 1.419, 1.255, -0.145};
    do{
      pose[0] = randomDouble(-0.12, 0.27);
      pose[1] = randomDouble(-1.0, -0.8);
      pose[2] = randomDouble(0.7, 1.0);
      // pose[3] = randomDouble(1.2, 1.6);
      // pose[4] = randomDouble(1.0, 1.4);
      // pose[5] = randomDouble(-0.0, -0.3);
    rmodel_->computeIK(pose, seed_start, start);
    }while (!cc_->isStateValid(start, prm_->verbose_collisions_, false, dist));
    printf("%f %f %f %f %f %f %f\n", start[0],start[1],start[2],start[3],start[4],start[5],start[6]);
    fprintf(fout, "    start:\n");
    fprintf(fout, "      rarm: %f %f %f %f %f %f %f\n", start[0],start[1],start[2],start[3],start[4],start[5],start[6]);
///////goal
    // vector<double> pose{0.456899, -0.117160, 0.979574, 1.419, 1.255, -0.145};
    do{
      pose[0] = randomDouble(0.45, 0.68);
      pose[1] = randomDouble(-0.5, 0.14);
      pose[2] = randomDouble(0.7, 1.0);
      // pose[3] = randomDouble(1.2, 1.6);
      // pose[4] = randomDouble(1.0, 1.4);
      // pose[5] = randomDouble(-0.0, -0.3);
    rmodel_->computeIK(pose, seed_goal, goal);
    }while (!cc_->isStateValid(goal, prm_->verbose_collisions_, false, dist));
    fprintf(fout, "    goal:\n");
    fprintf(fout, "      rarm: %f %f %f %f %f %f %f\n", goal[0],goal[1],goal[2],goal[3],goal[4],goal[5],goal[6]);   
    printf("%f %f %f %f %f %f %f\n", goal[0],goal[1],goal[2],goal[3],goal[4],goal[5],goal[6]);
    vector<double> left_arm{ 1.500,  1.000,  0.006, -1.400,  0.001, -0.744,  0.000};
    

    printf("start\n");
    pviz_.visualizeRobot(start, left_arm, body, 120, "state", 2, true);
    getchar();
    printf("goal\n");
    pviz_.visualizeRobot(goal, left_arm, body, 120, "state", 2, true);
    getchar();
      
    }
    fprintf(fout, "\n");
    fclose(fout);
    getchar();  
}
void EnvironmentOMPLROBARM3D::GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
  GetSuccs(SourceStateID, SuccIDV, CostV);  
  isTrueCost->resize(SuccIDV->size(), true);
  return;
}

void EnvironmentOMPLROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  double dist=0;
  int endeff[3]={0};
  int path_length=0, nchecks=0;
  std::vector<int> scoord(prm_->num_joints_,0);
  std::vector<double> pose(6,0), angles(prm_->num_joints_,0), source_angles(prm_->num_joints_,0);
  
  //clear the successor array
  SuccIDV->clear();
  CostV->clear();
  
  //goal state should be absorbing
  if(SourceStateID == pdata_.goal_entry->stateID)
    return;
  
  //get X, Y, Z for the state
  EnvOMPLROBARM3DHashEntry_t* parent_entry = pdata_.StateID2CoordTable[SourceStateID];
  
  if(int(parent_entry->coord.size()) < prm_->num_joints_)
    ROS_ERROR("Parent hash entry has broken coords. (# coords: %d)", int(parent_entry->coord.size()));
  
  //default coords of successor
  for(int i = 0; i < prm_->num_joints_; i++)
    scoord[i] = parent_entry->coord.at(i);

  //used for interpolated collision check
  coordToAngles(scoord, source_angles);

  //debugging
  //printf("Parent id %d given by angles %0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n", SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3], source_angles[4],source_angles[5],source_angles[6]);    

  std::vector<double> source_pose(6,0);
  rmodel_->computePlanningLinkFK(source_angles, source_pose);

  ROS_DEBUG_NAMED(prm_->expands_log_, "\nstate %d: %.2f %.2f %.2f %.2f %.2f %.2f %.2f endeff: %3d %3d %3d",SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2]);
  int valid = 1;
  
  std::vector<Action> actions;
  if(!as_->getActionSet(source_angles, actions))
  {
    ROS_WARN("Failed to get successors.");
    return;
  }
  ROS_DEBUG_NAMED(prm_->expands_log_, "[parent: %d] angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f xyz: %3d %3d %3d #_actions: %d heur: %d dist: %0.3f", SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2], int(actions.size()), getXYZHeuristic(SourceStateID, 1), double(bfs_->getDistance(parent_entry->xyz[0],parent_entry->xyz[1], parent_entry->xyz[2])) * grid_->getResolution());
  
  // check actions for validity
  for (int i = 0; i < int(actions.size()); ++i)
  {
    valid = 1;

    for(size_t j = 0; j < actions[i].size(); ++j)
    {
      ROS_DEBUG_NAMED(prm_->expands_log_,"[ succ: %d] angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", i, actions[i][j][0], actions[i][j][1], actions[i][j][2], actions[i][j][3], actions[i][j][4], actions[i][j][5], actions[i][j][6]);
      
      // check joint limits
      if(!rmodel_->checkJointLimits(actions[i][j]))
        valid = -1;
      
      //check for collisions
      if(!cc_->isStateValid(actions[i][j], pdata_.frames, prm_->verbose_collisions_, false, dist))
      {
        ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d dist: %0.3f is in collision.", i, dist);
        valid = -2;
      }

      if(valid < 1)
        break;
    }
    
    if(valid < 1)
      continue;
    
    std::vector<std::vector<double> > *path_out;
    // check for collisions along path from parent to first waypoint
    if(!cc_->isStateToStateValid(source_angles, actions[i][0], pdata_.frames, path_length, nchecks, dist, path_out))
    {
      ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
      valid = -3;
    }
    
    if(valid < 1)
      continue;
    
    //check for collisions between waypoints
    for(size_t j = 1; j < actions[i].size(); ++j)
    {
      if(!cc_->isStateToStateValid(actions[i][j-1], actions[i][j], pdata_.frames, path_length, nchecks, dist, path_out))
      {
        ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
        valid = -4;
        break;
      } 
    }
    
    if(valid < 1)
      continue;
    
    // compute coords
    anglesToCoord(actions[i].back(), scoord);
    
    // get the successor
    EnvOMPLROBARM3DHashEntry_t* succ_entry;
    bool succ_is_goal_state = false;
    
    // get pose of planning link
    if(!rmodel_->computePlanningLinkFK(actions[i].back(), pose))
      continue;

    // discretize planning link pose
    grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);
    
    //ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", int(i), actions[i].back()[0], actions[i].back()[1], actions[i].back()[2], actions[i].back()[3], actions[i].back()[4], actions[i].back()[5], actions[i].back()[6]);
    ROS_DEBUG_NAMED(prm_->expands_log_,"[ succ: %d] pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", int(i), pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    ROS_DEBUG_NAMED(prm_->expands_log_,"[ succ: %d] xyz: %d %d %d goal: %d %d %d (diff: %d %d %d)", int(i), endeff[0], endeff[1], endeff[2], pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2], abs(pdata_.goal_entry->xyz[0] - endeff[0]), abs(pdata_.goal_entry->xyz[1] - endeff[1]), abs(pdata_.goal_entry->xyz[2] - endeff[2]));
    
    //check if this state meets the goal criteria
    if(isGoalState(actions[i].back(), pdata_.goal))
    {
      succ_is_goal_state = true;
      for (int k = 0; k < prm_->num_joints_; k++)
      {
        pdata_.goal_entry->coord[k] = scoord[k];
        pdata_.goal_entry->xyz[0] = endeff[0];
        pdata_.goal_entry->xyz[1] = endeff[1];
        pdata_.goal_entry->xyz[2] = endeff[2];
        pdata_.goal_entry->state = actions[i].back();
        pdata_.goal_entry->dist = dist;
      }

    }
    //check if hash entry already exists, if not then create one
    if((succ_entry = getHashEntry(scoord, succ_is_goal_state)) == NULL)
    {
      succ_entry = createHashEntry(scoord, endeff);
      succ_entry->state = actions[i].back();
      succ_entry->dist = dist;
      ROS_DEBUG_NAMED(prm_->expands_log_, "%5i: action: %2d dist: %2d edge_distance_cost: %5d heur: %2d endeff: %3d %3d %3d", succ_entry->stateID, i, int(succ_entry->dist), cost(parent_entry,succ_entry, succ_is_goal_state), GetFromToHeuristic(succ_entry->stateID, pdata_.goal_entry->stateID), succ_entry->xyz[0],succ_entry->xyz[1],succ_entry->xyz[2]);
    }

    std::vector<double> succ_angles(7,0);
    coordToAngles(scoord, succ_angles);

    // std::vector<double> diff_angles(prm_->num_joints_,0);
    // for(int i = 0; i < 7; i++){
    //   diff_angles[i] = angles::shortest_angular_distance(source_angles[i],succ_angles[i]); 
    // }

    // double euclidean_diff = sqrt(diff_angles[0]*diff_angles[0] + diff_angles[1]*diff_angles[1] + diff_angles[2]*diff_angles[2] +
    //                            diff_angles[3]*diff_angles[3] + diff_angles[4]*diff_angles[4] + diff_angles[5]*diff_angles[5] +
    //                             diff_angles[6]*diff_angles[6] );

    // std::vector<double> succ_pose(6,0);
    // rmodel_->computePlanningLinkFK(succ_angles, succ_pose);

    // double pose_dist =  sqrt((source_pose[0] - succ_pose[0])*(source_pose[0] - succ_pose[0]) + 
    //                        (source_pose[1] - succ_pose[1])*(source_pose[1] - succ_pose[1]) +
    //                        (source_pose[2] - succ_pose[2])*(source_pose[2] - succ_pose[2]));

    // double scale = euclidean_diff/pose_dist;

    // scale_vec_.push_back(scale);
    
    //ROS_INFO("Euc is %f Pose dist is %f Scale %f",euclidean_diff, pose_dist, scale);
    
    //put successor on successor list with the proper cost
    SuccIDV->push_back(succ_entry->stateID);
    CostV->push_back(cost(parent_entry, succ_entry, succ_is_goal_state));
    //printf("Successor id %d given by angles %0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n", succ_entry->stateID, succ_angles[0], succ_angles[1], succ_angles[2], succ_angles[3], succ_angles[4], succ_angles[5], succ_angles[6]);
    //printf("The edge cost is %d\n", cost(parent_entry, succ_entry, succ_is_goal_state));
  }

  pdata_.expanded_states.push_back(SourceStateID);

  //double min = *std::min_element(scale_vec_.begin(), scale_vec_.end());

  //ROS_INFO("minimum is %f", min);

}

void EnvironmentOMPLROBARM3D::GetSuccs(int q_id, int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  double dist=0;
  int endeff[3]={0};
  int path_length=0, nchecks=0;
  std::vector<int> scoord(prm_->num_joints_,0);
  std::vector<double> pose(6,0), angles(prm_->num_joints_,0), source_angles(prm_->num_joints_,0);

  //clear the successor array
  SuccIDV->clear();
  CostV->clear();

  //goal state should be absorbing
  if(SourceStateID == pdata_.goal_entry->stateID)
    return;

  //get X, Y, Z for the state
  EnvOMPLROBARM3DHashEntry_t* parent_entry = pdata_.StateID2CoordTable[SourceStateID];

  if(int(parent_entry->coord.size()) < prm_->num_joints_)
    ROS_ERROR("Parent hash entry has broken coords. (# coords: %d)", int(parent_entry->coord.size()));
  //default coords of successor
  for(int i = 0; i < prm_->num_joints_; i++)
    scoord[i] = parent_entry->coord.at(i);

  //used for interpolated collision check
  coordToAngles(scoord, source_angles);

// base: 0.000000 0.000000 0.000000
//       spine: 0.100000
//       left: 2.135000 0.803000 1.732000 -1.905000 2.369000 -1.680000 1.398000
  vector<double> left_angles{ 1.500,  1.000,  0.006, -1.400,  0.001, -0.744,  0.000};
  BodyPose body;
  body.x = 0;
  body.y = 0;
  body.z = 0.100000;

  // pviz_.visualizeRobot(source_angles, left_angles, body, 120, "state", 2, true);
  // getchar();
  ROS_DEBUG_NAMED(prm_->expands_log_, "\nstate %d: %.2f %.2f %.2f %.2f %.2f %.2f %.2f  endeff: %3d %3d %3d",SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2]);
  
  int valid = 1;
  std::vector<Action> actions;
  if(!as_->getActionSet(q_id, source_angles, actions, true))
  {
    ROS_WARN("Failed to get successors.");
    return;
  }

  ROS_DEBUG_NAMED(prm_->expands_log_, "[parent: %d] angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f  xyz: %3d %3d %3d  #_actions: %d  heur: %d dist: %0.3f", SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2], int(actions.size()), getXYZHeuristic(SourceStateID, 1), double(bfs_->getDistance(parent_entry->xyz[0],parent_entry->xyz[1], parent_entry->xyz[2])) * grid_->getResolution());

  // check actions for validity
  for (int i = 0; i < int(actions.size()); ++i)
  {
    valid = 1;
    for(size_t j = 0; j < actions[i].size(); ++j)
    {
      ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d] angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f  %0.3f", i, actions[i][j][0], actions[i][j][1], actions[i][j][2], actions[i][j][3], actions[i][j][4], actions[i][j][5], actions[i][j][6]);

      // check joint limits
      if(!rmodel_->checkJointLimits(actions[i][j]))
        valid = -1;

      //check for collisions
      if(!cc_->isStateValid(actions[i][j], prm_->verbose_collisions_, false, dist))
      {
        ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d  dist: %0.3f is in collision.", i, dist);
        valid = -2;
      }

      if(valid < 1)
        break;
    }

    if(valid < 1)
      continue;

    std::vector<std::vector<double> > *path_out;
    // check for collisions along path from parent to first waypoint
    if(!cc_->isStateToStateValid(source_angles, actions[i][0], path_length, nchecks, dist))
    {
      ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d  dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
      valid = -3;
    }

    if(valid < 1)
      continue;

    // check for collisions between waypoints
    for(size_t j = 1; j < actions[i].size(); ++j)
    {
      //ROS_INFO("[ succ: %d] Checking interpolated path from waypoint %d to waypoint %d.", int(i), int(j-1), int(j));
      if(!cc_->isStateToStateValid(actions[i][j-1], actions[i][j], path_length, nchecks, dist))
      {
        ROS_DEBUG_NAMED(prm_->expands_log_, " succ: %2d  dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
        valid = -4;
        break;
      }
    }

    if(valid < 1)
      continue;
    // compute coords
    anglesToCoord(actions[i].back(), scoord);

    // get the successor
    EnvOMPLROBARM3DHashEntry_t* succ_entry;
    bool succ_is_goal_state = false;

    // get pose of planning link
    if(!rmodel_->computePlanningLinkFK(actions[i].back(), pose))
      continue;

    // discretize planning link pose
    grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);
    
    //ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", int(i), actions[i].back()[0], actions[i].back()[1], actions[i].back()[2], actions[i].back()[3], actions[i].back()[4], actions[i].back()[5], actions[i].back()[6]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d]   pose: %0.3f %0.3f %0.3f   %0.3f %0.3f %0.3f", int(i), pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d]    xyz: %d %d %d  goal: %d %d %d  (diff: %d %d %d)", int(i), endeff[0], endeff[1], endeff[2], pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2], abs(pdata_.goal_entry->xyz[0] - endeff[0]), abs(pdata_.goal_entry->xyz[1] - endeff[1]), abs(pdata_.goal_entry->xyz[2] - endeff[2]));

    //check if this state meets the goal criteria
    if(isGoalState(pose, pdata_.goal))
    {
      succ_is_goal_state = true;

      for (int k = 0; k < prm_->num_joints_; k++)
        pdata_.goal_entry->coord[k] = scoord[k];

      pdata_.goal_entry->xyz[0] = endeff[0];
      pdata_.goal_entry->xyz[1] = endeff[1];
      pdata_.goal_entry->xyz[2] = endeff[2];
      pdata_.goal_entry->state = actions[i].back();
      pdata_.goal_entry->dist = dist;
    }

    //check if hash entry already exists, if not then create one
    if((succ_entry = getHashEntry(scoord, succ_is_goal_state, true)) == NULL)
    {
      succ_entry = createHashEntry(scoord, endeff);
      succ_entry->state = actions[i].back();
      succ_entry->dist = dist;

      ROS_DEBUG_NAMED(prm_->expands_log_, "%5i: action: %2d dist: %2d edge_distance_cost: %5d heur: %2d endeff: %3d %3d %3d", succ_entry->stateID, i, int(succ_entry->dist), cost(parent_entry,succ_entry, succ_is_goal_state), GetFromToHeuristic(succ_entry->stateID, pdata_.goal_entry->stateID), succ_entry->xyz[0],succ_entry->xyz[1],succ_entry->xyz[2]);
    }

    // printf("succ id %d last_id %d\n", succ_entry->stateID, state_id_last);
    // if (succ_entry->stateID == state_id_last)
    // {
    //   printf("huurrrraaaaaaaaaa getsucc id %d \n",state_id_last);
    //   getchar();
    // }
    //put successor on successor list with the proper cost
    SuccIDV->push_back(succ_entry->stateID);
    CostV->push_back(cost(parent_entry, succ_entry, succ_is_goal_state));
  }

  pdata_.expanded_states.push_back(SourceStateID);
}
  
void EnvironmentOMPLROBARM3D::GetPreds(int SourceStateID, vector<int>* PredIDV, vector<int>* CostV)
{
}
void EnvironmentOMPLROBARM3D::GetPreds(int q_id, int SourceStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  double dist=0;
  int endeff[3]={0};
  int path_length=0, nchecks=0;
  std::vector<int> scoord(prm_->num_joints_,0);
  std::vector<double> pose(6,0), angles(prm_->num_joints_,0), source_angles(prm_->num_joints_,0);

  //clear the successor array
  PredIDV->clear();
  CostV->clear();

  //goal state should be absorbing
  if(SourceStateID == pdata_.start_entry->stateID)
    return;

  //get X, Y, Z for the state
  EnvOMPLROBARM3DHashEntry_t* parent_entry = pdata_.StateID2CoordTable[SourceStateID];

  if(int(parent_entry->coord.size()) < prm_->num_joints_)
    ROS_ERROR("Parent hash entry has broken coords. (# coords: %d)", int(parent_entry->coord.size()));
  //default coords of successor
  for(int i = 0; i < prm_->num_joints_; i++)
    scoord[i] = parent_entry->coord.at(i);

  //used for interpolated collision check
  coordToAngles(scoord, source_angles);

  vector<double> left_angles{ 1.500,  1.000,  0.006, -1.400,  0.001, -0.744,  0.000};
  BodyPose body;
  body.x = 0;
  body.y = 0;
  body.z = 0.100000;
  // if (q_id)
  // pviz_.visualizeRobot(source_angles, left_angles, body, 120, "state", 2, true);
  // getchar();

  ROS_DEBUG_NAMED(prm_->expands_log_, "\nstate %d: %.2f %.2f %.2f %.2f %.2f %.2f %.2f  endeff: %3d %3d %3d",SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2]);
 
  int valid = 1;
  std::vector<Action> actions;
  if(!as_->getActionSet(q_id, source_angles, actions, false))
  {
    ROS_WARN("Failed to get predecessors.");
    return;
  }

  ROS_DEBUG_NAMED(prm_->expands_log_, "[parent: %d] angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f  xyz: %3d %3d %3d  #_actions: %d  heur: %d dist: %0.3f", SourceStateID, source_angles[0],source_angles[1],source_angles[2],source_angles[3],source_angles[4],source_angles[5],source_angles[6], parent_entry->xyz[0],parent_entry->xyz[1],parent_entry->xyz[2], int(actions.size()), getXYZHeuristicBack(SourceStateID, 1), double(bfs_b->getDistance(parent_entry->xyz[0],parent_entry->xyz[1], parent_entry->xyz[2])) * grid_->getResolution());

  // check actions for validity
  for (int i = 0; i < int(actions.size()); ++i)
  {
    valid = 1;
    for(size_t j = 0; j < actions[i].size(); ++j)
    {     
      ROS_DEBUG_NAMED(prm_->expands_log_, "[ pred: %d] angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f  %0.3f", i, actions[i][j][0], actions[i][j][1], actions[i][j][2], actions[i][j][3], actions[i][j][4], actions[i][j][5], actions[i][j][6]);

      // check joint limits
      if(!rmodel_->checkJointLimits(actions[i][j]))
        valid = -1;
      //check for collisions
      if(!cc_->isStateValid(actions[i][j], prm_->verbose_collisions_, false, dist))
      {
        ROS_DEBUG_NAMED(prm_->expands_log_," pred: %2d  dist: %0.3f is in collision.", i, dist);
        valid = -2;
      }

      if(valid < 1)
        break;
    }

    if(valid < 1)
      continue;

    std::vector<std::vector<double> > *path_out;
    // check for collisions along path from parent to first waypoint
    if(!cc_->isStateToStateValid(source_angles, actions[i][0], path_length, nchecks, dist))
    {
      ROS_DEBUG_NAMED(prm_->expands_log_, " pred: %2d  dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
      valid = -3;
    }

    if(valid < 1)
      continue;

    // check for collisions between waypoints
    for(size_t j = 1; j < actions[i].size(); ++j)
    {
      //ROS_INFO("[ pred: %d] Checking interpolated path from waypoint %d to waypoint %d.", int(i), int(j-1), int(j));
      if(!cc_->isStateToStateValid(actions[i][j-1], actions[i][j], path_length, nchecks, dist))
      {
        ROS_DEBUG_NAMED(prm_->expands_log_, " pred: %2d  dist: %0.3f is in collision along interpolated path. (path_length: %d)", i, dist, path_length);
        valid = -4;
        break;
      }
    }

    if(valid < 1)
      continue;

    // compute coords
    anglesToCoord(actions[i].back(), scoord);

    // get the successor
    EnvOMPLROBARM3DHashEntry_t* pred_entry;
    bool pred_is_goal_state = false;

    // get pose of planning link
    if(!rmodel_->computePlanningLinkFK(actions[i].back(), pose))
      continue;

    // discretize planning link pose
    grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);
   
    //ROS_DEBUG_NAMED(prm_->expands_log_, "[ succ: %d] %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", int(i), actions[i].back()[0], actions[i].back()[1], actions[i].back()[2], actions[i].back()[3], actions[i].back()[4], actions[i].back()[5], actions[i].back()[6]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "[ pred: %d]   pose: %0.3f %0.3f %0.3f   %0.3f %0.3f %0.3f", int(i), pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    ROS_DEBUG_NAMED(prm_->expands_log_, "[ pred: %d]    xyz: %d %d %d  goal: %d %d %d  (diff: %d %d %d)", int(i), endeff[0], endeff[1], endeff[2], pdata_.start_entry->xyz[0], pdata_.start_entry->xyz[1], pdata_.start_entry->xyz[2], abs(pdata_.start_entry->xyz[0] - endeff[0]), abs(pdata_.start_entry->xyz[1] - endeff[1]), abs(pdata_.start_entry->xyz[2] - endeff[2]));

    //check if this state meets the goal criteria
    if(isStartState(pose, pdata_.start))
    {
      pred_is_goal_state = true;

      for (int k = 0; k < prm_->num_joints_; k++)
        pdata_.start_entry->coord[k] = scoord[k];

      pdata_.start_entry->xyz[0] = endeff[0];
      pdata_.start_entry->xyz[1] = endeff[1];
      pdata_.start_entry->xyz[2] = endeff[2];
      pdata_.start_entry->state = actions[i].back();
      pdata_.start_entry->dist = dist;
    }

    //check if hash entry already exists, if not then create one
    if((pred_entry = getHashEntry(scoord, pred_is_goal_state, false)) == NULL)
    {
      pred_entry = createHashEntry(scoord, endeff);
      pred_entry->state = actions[i].back();
      pred_entry->dist = dist;

      ROS_DEBUG_NAMED(prm_->expands_log_, "%5i: action: %2d dist: %2d edge_distance_cost: %5d heur: %2d endeff: %3d %3d %3d", pred_entry->stateID, i, int(pred_entry->dist), cost(parent_entry,pred_entry, pred_is_goal_state), GetFromToHeuristicBack(pred_entry->stateID, pdata_.start_entry->stateID), pred_entry->xyz[0],pred_entry->xyz[1],pred_entry->xyz[2]);
    }
    // printf("pred id %d last_id %d\n", pred_entry->stateID, state_id_last_b);
    // if (pred_entry->stateID == state_id_last_b)
    // {
    //   printf("huurrrraaaaaaaaaa getpreds id %d\n", state_id_last_b);
    //   getchar();
    // }
    //put successor on successor list with the proper cost
    PredIDV->push_back(pred_entry->stateID);
    CostV->push_back(cost(parent_entry, pred_entry, pred_is_goal_state));
  }
  pdata_.expanded_states.push_back(SourceStateID);
}

bool EnvironmentOMPLROBARM3D::AreEquivalent(int StateID1, int StateID2)
{
  ROS_ERROR("ERROR in pdata_... function: AreEquivalent is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentOMPLROBARM3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  ROS_ERROR("ERROR in pdata_..function: SetAllActionsandOutcomes is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentOMPLROBARM3D::SetAllPreds(CMDPSTATE* state)
{
  ROS_ERROR("ERROR in pdata_... function: SetAllPreds is undefined\n");
  throw new SBPL_Exception();
}

/////////////////////////////////////////////////////////////////////////////
//                      End of SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

void EnvironmentOMPLROBARM3D::printHashTableHist()
{
  int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

  for(int  j = 0; j < pdata_.HashTableSize; j++)
  {
    if((int)pdata_.Coord2StateIDHashTable[j].size() == 0)
      s0++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 50)
      s1++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 100)
      s50++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 200)
      s100++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 300)
      s200++;
    else if((int)pdata_.Coord2StateIDHashTable[j].size() < 400)
      s300++;
    else
      slarge++;
  }
  ROS_DEBUG("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d",
      s0,s1, s50, s100, s200,s300,slarge);
}

EnvOMPLROBARM3DHashEntry_t* EnvironmentOMPLROBARM3D::getHashEntry(const std::vector<int> &coord, bool bIsGoal)
{
  //if it is goal
  if(bIsGoal)
    return pdata_.goal_entry;

  int binid = getHashBin(coord);

#if DEBUG
  if ((int)pdata_.Coord2StateIDHashTable[binid].size() > 500)
  {
    ROS_WARN("WARNING: Hash table has a bin %d (coord0=%d) of size %d", 
        binid, coord[0], int(pdata_.Coord2StateIDHashTable[binid].size()));
    printHashTableHist();
  }
#endif

  //iterate over the states in the bin and select the perfect match
  for(int ind = 0; ind < (int)pdata_.Coord2StateIDHashTable[binid].size(); ind++)
  {
    int j = 0;

    for(j = 0; j < int(coord.size()); j++)
    {
      if(pdata_.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) 
        break;
    }

    if (j == int(coord.size()))
      return pdata_.Coord2StateIDHashTable[binid][ind];
  }

  return NULL;
}

EnvOMPLROBARM3DHashEntry_t* EnvironmentOMPLROBARM3D::getHashEntry(const std::vector<int> &coord, bool bIsGoal, bool forward)
{
  //if it is goal
  if(bIsGoal)
  {
    if (forward)
      return pdata_.goal_entry;
    else
      return pdata_.start_entry;
  }

  int binid = getHashBin(coord);

#if DEBUG
  if ((int)pdata_.Coord2StateIDHashTable[binid].size() > 500)
  {
    ROS_WARN("WARNING: Hash table has a bin %d (coord0=%d) of size %d", 
        binid, coord[0], int(pdata_.Coord2StateIDHashTable[binid].size()));
    printHashTableHist();
  }
#endif

  //iterate over the states in the bin and select the perfect match
  for(int ind = 0; ind < (int)pdata_.Coord2StateIDHashTable[binid].size(); ind++)
  {
    int j = 0;

    for(j = 0; j < int(coord.size()); j++)
    {
      if(pdata_.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) 
        break;
    }

    if (j == int(coord.size()))
      return pdata_.Coord2StateIDHashTable[binid][ind];
  }
  return NULL;
}

EnvOMPLROBARM3DHashEntry_t* EnvironmentOMPLROBARM3D::createHashEntry(const std::vector<int> &coord, int endeff[3])
{
  int i;
  EnvOMPLROBARM3DHashEntry_t* HashEntry = new EnvOMPLROBARM3DHashEntry_t;

  HashEntry->coord = coord;

  memcpy(HashEntry->xyz, endeff, 3*sizeof(int));

  // assign a stateID to HashEntry to be used 
  HashEntry->stateID = pdata_.StateID2CoordTable.size();

  //insert into the tables
  pdata_.StateID2CoordTable.push_back(HashEntry);

  //get the hash table bin
  i = getHashBin(HashEntry->coord);

  //insert the entry into the bin
  pdata_.Coord2StateIDHashTable[i].push_back(HashEntry);

  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
  {
    StateID2IndexMapping[HashEntry->stateID][i] = -1;
  }

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
  {
    ROS_ERROR("ERROR in Env... function: last state has incorrect stateID");
    throw new SBPL_Exception();
  }

  return HashEntry;
}

bool EnvironmentOMPLROBARM3D::getGoalFA(double &fa)
{
  fa = pdata_.goal.free_angle;

  if(pdata_.goal.type == GoalType::XYZ_RPY_FA_GOAL)
    return true;
  else
    return false;
}

int EnvironmentOMPLROBARM3D::cost(EnvOMPLROBARM3DHashEntry_t* HashEntry1, EnvOMPLROBARM3DHashEntry_t* HashEntry2, bool bState2IsGoal)
{
  std::vector<double> from_config(prm_->num_joints_,0);
  coordToAngles(HashEntry1->coord, from_config);

  std::vector<double> to_config(prm_->num_joints_,0);
  coordToAngles(HashEntry2->coord, to_config);

  ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(7));

  const ompl::base::State *from_config_state = space->allocState();
  const ompl::base::State *to_config_state = space->allocState();

  for(int i = 0; i < prm_->num_joints_; i++)
  {
    from_config_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = from_config[i];
    //printf("SBPL:parent %f\n", from_config[i]);
  }

  for(int i = 0; i < prm_->num_joints_; i++)
  {
    to_config_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = to_config[i];
    //printf("SBPL:child %f\n", to_config[i]);
  }
  
  //return prm_->cost_multiplier_;
  return GetContEdgeCost(from_config_state, to_config_state);
}

bool EnvironmentOMPLROBARM3D::initEnvironment()
{
  // initialize environment data
  pdata_.init();

  //create empty start & goal states
  int endeff[3] = {0};
  std::vector<int> coord(prm_->num_joints_,0);
  pdata_.start_entry = createHashEntry(coord, endeff);
  pdata_.goal_entry = createHashEntry(coord, endeff);

  //compute the cost per cell to be used by heuristic
  computeCostPerCell();

  //initialize BFS
  int dimX, dimY, dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);
  bfs_ = new BFS_3D(dimX, dimY, dimZ);
  bfs_b = new BFS_3D(dimX, dimY, dimZ);

  //set heuristic function pointer
  getHeuristic_ = &sbpl_arm_planner::EnvironmentOMPLROBARM3D::getXYZHeuristic;
  getHeuristicBack_ = &sbpl_arm_planner::EnvironmentOMPLROBARM3D::getXYZHeuristicBack;

  //set 'environment is initialized' flag
  prm_->ready_to_plan_ = true; 
  ROS_INFO("[env] Environment has been initialized.");
  return true;
}

bool EnvironmentOMPLROBARM3D::isGoalState(const std::vector<double> &pose, GoalConstraint &goal)
{

  if(goal.type == XYZ_RPY_GOAL)
  {
    if(fabs(pose[0]-goal.pose[0]) <= goal.xyz_tolerance[0] && 
        fabs(pose[1]-goal.pose[1]) <= goal.xyz_tolerance[1] && 
        fabs(pose[2]-goal.pose[2]) <= goal.xyz_tolerance[2])
    {
      //log the amount of time required for the search to get close to the goal
      if(!pdata_.near_goal)
      {
        pdata_.time_to_goal_region = (clock() - pdata_.t_start) / (double)CLOCKS_PER_SEC;
        pdata_.near_goal = true;
        printf("Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %.4f sec. (after %d expansions)\n", pose[0],pose[1],pose[2],goal.xyz_tolerance[0], goal.pose[0], goal.pose[1], goal.pose[2], pdata_.time_to_goal_region,(int)pdata_.expanded_states.size());
      }

      if(fabs(pose[3]-goal.pose[3]) <= goal.rpy_tolerance[0] && 
          fabs(pose[4]-goal.pose[4]) <= goal.rpy_tolerance[1] && 
          fabs(pose[5]-goal.pose[5]) <= goal.rpy_tolerance[2]){
        return true;
      }
    }
  }
  else if(goal.type == XYZ_GOAL)
  {
    if(fabs(pose[0]-goal.pose[0]) <= goal.xyz_tolerance[0] && 
        fabs(pose[1]-goal.pose[1]) <= goal.xyz_tolerance[1] && 
        fabs(pose[2]-goal.pose[2]) <= goal.xyz_tolerance[2])
      return true;
  }
  else if(goal.type == SEVEN_DOF_GOAL)
  {
      std::vector<double> goal_angles;
      getGoalAngles(goal_angles);
      
      if(fabs(pose[0]-goal_angles[0]) <= goal.seven_dof_tolerance[0] && 
      fabs(pose[1]-goal_angles[1]) <= goal.seven_dof_tolerance[1] && 
      fabs(pose[2]-goal_angles[2]) <= goal.seven_dof_tolerance[2] &&
      fabs(pose[3]-goal_angles[3]) <= goal.seven_dof_tolerance[3] &&
      fabs(pose[4]-goal_angles[4]) <= goal.seven_dof_tolerance[4] &&
      fabs(pose[5]-goal_angles[5]) <= goal.seven_dof_tolerance[5] &&
      fabs(pose[6]-goal_angles[6]) <= goal.seven_dof_tolerance[6])
      {
        if(!pdata_.near_goal)
        {
          pdata_.near_goal = true;
          ROS_INFO("Search is near the goal region");
          return true;
        } 
      }

  }
  else
    ROS_ERROR("Unknown goal type.");

  return false;
}

bool EnvironmentOMPLROBARM3D::isStartState(const std::vector<double> &pose, GoalConstraint &start)
{
  if(start.type == XYZ_RPY_GOAL)
  {
    if(fabs(pose[0]-start.pose[0]) <= start.xyz_tolerance[0] && 
        fabs(pose[1]-start.pose[1]) <= start.xyz_tolerance[1] && 
        fabs(pose[2]-start.pose[2]) <= start.xyz_tolerance[2])
    {
      //log the amount of time required for the search to get close to the goal
      if(!pdata_.near_start)
      {
        pdata_.time_to_start_region = (clock() - pdata_.t_start) / (double)CLOCKS_PER_SEC;
        pdata_.near_start = true;
        printf("Search is at %0.2f %0.2f %0.2f, within %0.3fm of the start (%0.2f %0.2f %0.2f) after %.4f sec. (after %d expansions)\n", pose[0],pose[1],pose[2],start.xyz_tolerance[0], start.pose[0], start.pose[1], start.pose[2], pdata_.time_to_start_region,(int)pdata_.expanded_states.size());
      }

      if(fabs(pose[3]-start.pose[3]) <= start.rpy_tolerance[0] && 
          fabs(pose[4]-start.pose[4]) <= start.rpy_tolerance[1] && 
          fabs(pose[5]-start.pose[5]) <= start.rpy_tolerance[2])
          {
              return true;
          }
        
    }
  }
  else if(start.type == XYZ_GOAL)
  {
    if(fabs(pose[0]-start.pose[0]) <= start.xyz_tolerance[0] && 
        fabs(pose[1]-start.pose[1]) <= start.xyz_tolerance[1] && 
        fabs(pose[2]-start.pose[2]) <= start.xyz_tolerance[2])
      return true;
  }
  else
    ROS_ERROR("Unknown start type.");

  return false;
}

int EnvironmentOMPLROBARM3D::getActionCost(const std::vector<double> &from_config, const std::vector<double> &to_config, int dist)
{


  int num_prims = 0, cost = 0;
  double diff = 0, max_diff = 0;

  if(from_config.size() != to_config.size())
    return -1;

/* NOTE: Not including forearm roll OR wrist roll movement to calculate mprim cost */

  for(size_t i = 0; i < 6; i++)
  {
    if(i == 4)
      continue;

    diff = fabs(angles::shortest_angular_distance(from_config[i], to_config[i]));
    if(max_diff < diff)
      max_diff = diff;
  }

  num_prims = max_diff / prm_->max_mprim_offset_ + 0.5;
  cost = num_prims * prm_->cost_multiplier_;

//   ROS_DEBUG_NAMED("search", "from: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f", angles::normalize_angle(from_config[0]),angles::normalize_angle(from_config[1]),angles::normalize_angle(from_config[2]),angles::normalize_angle(from_config[3]),angles::normalize_angle(from_config[4]),angles::normalize_angle(from_config[5]),angles::normalize_angle(from_config[6]));
//   ROS_DEBUG_NAMED("search", "  to: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f diff: %0.2f num_prims: %d cost: %d (mprim_size: %0.3f)", to_config[0],to_config[1],to_config[2],to_config[3],to_config[4],to_config[5],to_config[6], max_diff, num_prims, cost, prm_->max_mprim_offset_);
  
   return cost;
}

int EnvironmentOMPLROBARM3D::getEdgeCost(int FromStateID, int ToStateID)
{
#if DEBUG
  if(FromStateID >= (int)pdata_.StateID2CoordTable.size() 
      || ToStateID >= (int)pdata_.StateID2CoordTable.size())
  {
    ROS_ERROR("ERROR in pdata_... function: stateID illegal\n");
    throw new SBPL_Exception();
  }
#endif

  //get X, Y for the state
  EnvOMPLROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[FromStateID];
  EnvOMPLROBARM3DHashEntry_t* ToHashEntry = pdata_.StateID2CoordTable[ToStateID];

  return cost(FromHashEntry, ToHashEntry, false);
}

bool EnvironmentOMPLROBARM3D::setStartConfiguration(const std::vector<double> angles)
{
  double dist=0;
  int x,y,z;
  std::vector<double> pose(6,0);

  if(int(angles.size()) < prm_->num_joints_)
  {
    ROS_ERROR("Start state does not contain enough enough joint positions.");
    return false;
  }

  //get joint positions of starting configuration
  if(!rmodel_->computePlanningLinkFK(angles, pose))
    ROS_WARN("Unable to compute forward kinematics for initial robot state. Attempting to plan anyway.");
  ROS_INFO("[start]             angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]); 
  ROS_INFO("[start] planning_link pose:   xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

  //check joint limits of starting configuration but plan anyway
  if(!rmodel_->checkJointLimits(angles))
    ROS_WARN("Starting configuration violates the joint limits. Attempting to plan anyway.");

  //check if the start configuration is in collision but plan anyway
  if(!cc_->isStateValid(angles, prm_->verbose_collisions_, false, dist))
  {
    ROS_WARN("[env] The starting configuration is in collision. Attempting to plan anyway. (distance to nearest obstacle %0.2fm)", double(dist)*grid_->getResolution());
  }

  //get arm position in environment
  anglesToCoord(angles, pdata_.start_entry->coord);
  grid_->worldToGrid(pose[0],pose[1],pose[2],x,y,z);
  pdata_.start_entry->xyz[0] = (int)x;
  pdata_.start_entry->xyz[1] = (int)y;
  pdata_.start_entry->xyz[2] = (int)z;
  ROS_INFO("[start] coord: %d %d %d %d %d %d %d   pose: %d %d %d", pdata_.start_entry->coord[0], pdata_.start_entry->coord[1], pdata_.start_entry->coord[2], pdata_.start_entry->coord[3], pdata_.start_entry->coord[4], pdata_.start_entry->coord[5], pdata_.start_entry->coord[6], x, y, z);
  return true;
}

bool EnvironmentOMPLROBARM3D::setGoalConfiguration(const std::vector<double> angles)
{
  double dist=0;
  int x,y,z;
  std::vector<double> pose(6,0);

  if(int(angles.size()) < prm_->num_joints_)
  {
    ROS_ERROR("Goal state does not contain enough enough joint positions.");
    return false;
  }

  //get joint positions of Goal configuration
  if(!rmodel_->computePlanningLinkFK(angles, pose))
    ROS_WARN("Unable to compute forward kinematics for initial robot state. Attempting to plan anyway.");
  ROS_INFO("[goal]             angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]); 
  ROS_INFO("[goal] planning_link pose:   xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);


  //check joint limits of Goal configuration but plan anyway
  if(!rmodel_->checkJointLimits(angles))
    ROS_WARN("Goal configuration violates the joint limits. Attempting to plan anyway.");

  //check if the Goal configuration is in collision but plan anyway
  if(!cc_->isStateValid(angles, prm_->verbose_collisions_, false, dist))
  {
    ROS_WARN("[env] The goal configuration is in collision. Attempting to plan anyway. (distance to nearest obstacle %0.2fm)", double(dist)*grid_->getResolution());
  }

  ompl::base::StateSpacePtr space(si_->getStateSpace());
  ompl::base::State *ompl_goal = space->allocState();

  ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = angles[0];
  ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = angles[1];
  ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = angles[2];
  ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = angles[3];
  ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = angles[4];
  ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = angles[5];
  ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = angles[6];

  if(!si_->isValid(ompl_goal))
  {
    ROS_WARN("[ompl_env] The goal configuration is in collision. Attempting to plan anyway. (distance to nearest obstacle %0.2fm)", double(dist)*grid_->getResolution());
  }

  //get arm position in environment
  anglesToCoord(angles, pdata_.goal_entry->coord);
  grid_->worldToGrid(pose[0],pose[1],pose[2],x,y,z);
  pdata_.goal_entry->xyz[0] = (int)x;
  pdata_.goal_entry->xyz[1] = (int)y;
  pdata_.goal_entry->xyz[2] = (int)z;
  ROS_INFO("[goal] coord: %d %d %d %d %d %d %d pose: %d %d %d", pdata_.goal_entry->coord[0], pdata_.goal_entry->coord[1], pdata_.goal_entry->coord[2], pdata_.goal_entry->coord[3], pdata_.goal_entry->coord[4], pdata_.goal_entry->coord[5], pdata_.goal_entry->coord[6], x, y, z);
  return true;
}

bool EnvironmentOMPLROBARM3D::getGoalAngles(std::vector<double> & angles)
{
  coordToAngles(pdata_.goal_entry->coord, angles);
  return true;
}

bool EnvironmentOMPLROBARM3D::setGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances)
{
  //goals: {{x1,y1,z1,r1,p1,y1,is_6dof},{x2,y2,z2,r2,p2,y2,is_6dof}...}

  if(!prm_->ready_to_plan_)
  {
    ROS_ERROR("Cannot set goal position because environment is not initialized.");
    return false;
  }

  if(goals.empty())
  {
    ROS_ERROR("[setGoalPosition] No goal constraint set.");
    return false;
  }

  pdata_.goal.pose.resize(6,0);
  pdata_.goal.pose[0] = goals[0][0];
  pdata_.goal.pose[1] = goals[0][1];
  pdata_.goal.pose[2] = goals[0][2];
  pdata_.goal.pose[3] = goals[0][3];
  pdata_.goal.pose[4] = goals[0][4];
  pdata_.goal.pose[5] = goals[0][5];
  pdata_.goal.xyz_tolerance[0] = tolerances[0][0];
  pdata_.goal.xyz_tolerance[1] = tolerances[0][1];
  pdata_.goal.xyz_tolerance[2] = tolerances[0][2];
  pdata_.goal.rpy_tolerance[0] = tolerances[0][3];
  pdata_.goal.rpy_tolerance[1] = tolerances[0][4];
  pdata_.goal.rpy_tolerance[2] = tolerances[0][5];
  pdata_.goal.seven_dof_tolerance[0] = tolerances[0][6];
  pdata_.goal.seven_dof_tolerance[1] = tolerances[0][7];
  pdata_.goal.seven_dof_tolerance[2] = tolerances[0][8];
  pdata_.goal.seven_dof_tolerance[3] = tolerances[0][9];
  pdata_.goal.seven_dof_tolerance[4] = tolerances[0][10];
  pdata_.goal.seven_dof_tolerance[5] = tolerances[0][11];
  pdata_.goal.seven_dof_tolerance[6] = tolerances[0][12];
  pdata_.goal.type = goals[0][6];


  // set goal hash entry
  grid_->worldToGrid(goals[0][0], goals[0][1], goals[0][2], pdata_.goal_entry->xyz[0],pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2]);

  for(int i = 0; i < prm_->num_joints_; i++)
    pdata_.goal_entry->coord[i] = 0;

  ROS_DEBUG_NAMED(prm_->expands_log_, "time: %f", clock() / (double)CLOCKS_PER_SEC);
  ROS_DEBUG_NAMED(prm_->expands_log_, "A new goal has been set.");
  ROS_DEBUG_NAMED(prm_->expands_log_, "grid: %d %d %d (cells)  xyz: %.2f %.2f %.2f (meters)  (tol: %.3f) rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3f)", pdata_.goal_entry->xyz[0],pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2], pdata_.goal.pose[0], pdata_.goal.pose[1], pdata_.goal.pose[2], pdata_.goal.xyz_tolerance[0], pdata_.goal.pose[3], pdata_.goal.pose[4], pdata_.goal.pose[5], pdata_.goal.rpy_tolerance[0]);

  // push obstacles into bfs grid
  ros::WallTime start = ros::WallTime::now();
  start = ros::WallTime::now();
  int dimX, dimY, dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);
  int walls=0;
  for (int z = 0; z < dimZ - 2; z++)
    for (int y = 0; y < dimY - 2; y++)
      for (int x = 0; x < dimX - 2; x++)
        if(grid_->getDistance(x,y,z) <= prm_->planning_link_sphere_radius_)
        {
          bfs_->setWall(x + 1, y + 1, z + 1); //, true);
          walls++;
        }
  double set_walls_time = (ros::WallTime::now() - start).toSec();
  ROS_INFO("[env] %0.5fsec to set walls in new bfs. (%d walls (%0.3f percent))", set_walls_time, walls, double(walls)/double(dimX*dimY*dimZ));

  /*
  start = ros::WallTime::now();
  ROS_ERROR("sphere radius: %0.3fm  %dcells", prm_->planning_link_sphere_radius_, int(prm_->planning_link_sphere_radius_/grid_->getResolution() + 0.5));
  setDistanceField(bfs_, grid_->getDistanceFieldPtr(), int(prm_->planning_link_sphere_radius_/grid_->getResolution() + 0.5));
  ROS_INFO("[env] %0.5fsec to set walls in bfs.", (ros::WallTime::now() - start).toSec());
  */
  if((pdata_.goal_entry->xyz[0] < 0) || (pdata_.goal_entry->xyz[1] < 0) || (pdata_.goal_entry->xyz[2] < 0))
  {
    ROS_ERROR("Goal is out of bounds. Can't run BFS with {%d %d %d} as start.", pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2]);
    return false;
  }
  bfs_->run(pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2]);
  //bfs_->configure(pdata_.goal_entry->xyz[0], pdata_.goal_entry->xyz[1], pdata_.goal_entry->xyz[2]);

  //getVisualization("bfs_walls");
  //getVisualization("bfs_values");

  pdata_.near_goal = false; 
  pdata_.t_start = clock();
  return true;
}

bool EnvironmentOMPLROBARM3D::setStartPosition(const std::vector <std::vector<double> > &starts, const std::vector<std::vector<double> > &tolerances)
{
  //starts: {{x1,y1,z1,r1,p1,y1,is_6dof},{x2,y2,z2,r2,p2,y2,is_6dof}...}

  if(!prm_->ready_to_plan_)
  {
    ROS_ERROR("Cannot set start position because environment is not initialized.");
    return false;
  }

  if(starts.empty())
  {
    ROS_ERROR("[setstartPosition] No start constraint set.");
    return false;
  }

  pdata_.start.pose.resize(6,0);
  pdata_.start.pose[0] = starts[0][0];
  pdata_.start.pose[1] = starts[0][1];
  pdata_.start.pose[2] = starts[0][2];
  pdata_.start.pose[3] = starts[0][3];
  pdata_.start.pose[4] = starts[0][4];
  pdata_.start.pose[5] = starts[0][5];
  pdata_.start.xyz_tolerance[0] = tolerances[0][0];
  pdata_.start.xyz_tolerance[1] = tolerances[0][1];
  pdata_.start.xyz_tolerance[2] = tolerances[0][2];
  pdata_.start.rpy_tolerance[0] = tolerances[0][3];
  pdata_.start.rpy_tolerance[1] = tolerances[0][4];
  pdata_.start.rpy_tolerance[2] = tolerances[0][5];
  pdata_.start.type = starts[0][6];


  // set start hash entry
  grid_->worldToGrid(starts[0][0], starts[0][1], starts[0][2], pdata_.start_entry->xyz[0],pdata_.start_entry->xyz[1], pdata_.start_entry->xyz[2]);

  // for(int i = 0; i < prm_->num_joints_; i++)
  //   pdata_.start_entry->coord[i] = 1;

  ROS_DEBUG_NAMED(prm_->expands_log_, "time: %f", clock() / (double)CLOCKS_PER_SEC);
  ROS_DEBUG_NAMED(prm_->expands_log_, "A new start has been set.");
  ROS_DEBUG_NAMED(prm_->expands_log_, "grid: %d %d %d (cells)  xyz: %.2f %.2f %.2f (meters)  (tol: %.3f) rpy: %1.2f %1.2f %1.2f (radians) (tol: %.3f)", pdata_.start_entry->xyz[0],pdata_.start_entry->xyz[1], pdata_.start_entry->xyz[2], pdata_.start.pose[0], pdata_.start.pose[1], pdata_.start.pose[2], pdata_.start.xyz_tolerance[0], pdata_.start.pose[3], pdata_.start.pose[4], pdata_.start.pose[5], pdata_.start.rpy_tolerance[0]);


  // push obstacles into bfs grid
  ros::WallTime start = ros::WallTime::now();
  start = ros::WallTime::now();
  int dimX, dimY, dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);
  int walls=0;
  for (int z = 0; z < dimZ - 2; z++)
    for (int y = 0; y < dimY - 2; y++)
      for (int x = 0; x < dimX - 2; x++)
        if(grid_->getDistance(x,y,z) <= prm_->planning_link_sphere_radius_)
        {
          bfs_b->setWall(x + 1, y + 1, z + 1); //, true);
          walls++;
        }
  double set_walls_time = (ros::WallTime::now() - start).toSec();
  ROS_INFO("[env] %0.5fsec to set walls in new bfs. (%d walls (%0.3f percent))", set_walls_time, walls, double(walls)/double(dimX*dimY*dimZ));

  /*
  start = ros::WallTime::now();
  ROS_ERROR("sphere radius: %0.3fm  %dcells", prm_->planning_link_sphere_radius_, int(prm_->planning_link_sphere_radius_/grid_->getResolution() + 0.5));
  setDistanceField(bfs_, grid_->getDistanceFieldPtr(), int(prm_->planning_link_sphere_radius_/grid_->getResolution() + 0.5));
  ROS_INFO("[env] %0.5fsec to set walls in bfs.", (ros::WallTime::now() - start).toSec());
  */
  if((pdata_.start_entry->xyz[0] < 0) || (pdata_.start_entry->xyz[1] < 0) || (pdata_.start_entry->xyz[2] < 0))
  {
    ROS_ERROR("start is out of bounds. Can't run BFS with {%d %d %d} as goal.", pdata_.start_entry->xyz[0], pdata_.start_entry->xyz[1], pdata_.start_entry->xyz[2]);
    return false;
  }
  bfs_b->run(pdata_.start_entry->xyz[0], pdata_.start_entry->xyz[1], pdata_.start_entry->xyz[2]);
  //bfs_->configure(pdata_.start_entry->xyz[0], pdata_.start_entry->xyz[1], pdata_.start_entry->xyz[2]);

  pdata_.near_start = false; 
  pdata_.t_start = clock();

  state_id_last = pdata_.goal_entry->stateID;
  state_id_last_b = pdata_.start_entry->stateID;
  return true;
}

void EnvironmentOMPLROBARM3D::StateID2Angles(int stateID, std::vector<double> &angles)
{
  EnvOMPLROBARM3DHashEntry_t* HashEntry = pdata_.StateID2CoordTable[stateID];

  if(stateID == pdata_.goal_entry->stateID)
    coordToAngles(pdata_.goal_entry->coord, angles);
  else
    coordToAngles(HashEntry->coord, angles);

  for (size_t i = 0; i < angles.size(); i++)
  {
    if(angles[i] >= M_PI)
      angles[i] = -2.0*M_PI + angles[i];
  }
}

void EnvironmentOMPLROBARM3D::printJointArray(FILE* fOut, EnvOMPLROBARM3DHashEntry_t* HashEntry, bool bGoal, bool bVerbose)
{
  std::vector<double> angles(prm_->num_joints_,0);

  //if(bGoal)
  //  coordToAngles(pdata_.goal_entry->coord, angles);
  //else
    coordToAngles(HashEntry->coord, angles);

  std::vector<double> pose(6, 0);
  rmodel_->computePlanningLinkFK(angles, pose);

  if(bVerbose)
    fprintf(fOut, "angles: ");

  for(int i = 0; i < int(angles.size()); i++)
  {
    //if(i > 0)
    //  fprintf(fOut, "%-.3f ", angles[i]-angles[i-1]);
    //else
      fprintf(fOut, "%-.3f ", angles[i]);
  }
  fprintf(fOut, "\n");

  if(bVerbose)
    fprintf(fOut, "pose: ");

  for(int i = 0; i < int(pose.size()); i++)
  {
    fprintf(fOut, "%-.3f ", pose[i]);
  }
  fprintf(fOut, "\n");
}

void EnvironmentOMPLROBARM3D::getExpandedStates(std::vector<std::vector<double> >* states)
{
  std::vector<double> angles(prm_->num_joints_,0);
  std::vector<double> state(7,0); // {x,y,z,r,p,y,heur}

  for(size_t i = 0; i < pdata_.expanded_states.size(); ++i)
  {
    StateID2Angles(pdata_.expanded_states[i],angles);
    rmodel_->computePlanningLinkFK(angles,state);
    state[6] = pdata_.StateID2CoordTable[pdata_.expanded_states[i]]->heur;
    states->push_back(state);
    ROS_DEBUG("[%d] id: %d  xyz: %0.2f %0.2f %0.2f", int(i), pdata_.expanded_states[i], state[0], state[1], state[2]);
  }
}

void EnvironmentOMPLROBARM3D::computeCostPerCell()
{
  ROS_ERROR("Fill in function to computeCostPerCell()");
  prm_->cost_per_cell_ = 1;

  /*
  int largest_action=0;
  double gridcell_size, eucl_dist, max_dist = 0;
  std::vector<double> pose(6,0), start_pose(6,0), angles(prm_->num_joints_,0), start_angles(prm_->num_joints_,0);

  gridcell_size = grid_->getResolution();

  //starting at zeroed angles, find end effector position after each action
  rmodel_->computePlanningLinkFK(start_angles, start_pose);

  //iterate through all possible actions and find the one with the minimum cost per cell
  for (int i = 0; i < prm_->num_mprims_; i++)
  {
    for(int j = 0; j < prm_->num_joints_; j++)
      angles[j] = DEG2RAD(prm_->mprims_[i][j]);

    //starting at zeroed angles, find end effector position after each action
    if(!rmodel_->computePlanningLinkFK(angles, pose))
    {
      ROS_WARN("[env] Failed to compute cost per cell because forward kinematics is returning an error.");
      return;
    }

    eucl_dist = sqrt((start_pose[0]-pose[0])*(start_pose[0]-pose[0]) +
        (start_pose[1]-pose[1])*(start_pose[1]-pose[1]) +
        (start_pose[2]-pose[2])*(start_pose[2]-pose[2]));

    if (eucl_dist > max_dist)
    {
      max_dist = eucl_dist;
      largest_action = i;
    }
  }

  prm_->setCellCost(int(prm_->cost_multiplier_ / (max_dist/gridcell_size)));

  prm_->cost_per_meter_ = int(prm_->cost_per_cell_ / gridcell_size);

  ROS_INFO("[env] max_dist_traveled_per_smp: %0.3fm  cost per cell: %d  cost per meter: %d  (type: jointspace)", max_dist, prm_->cost_per_cell_,prm_->cost_per_meter_);
  */
}

int EnvironmentOMPLROBARM3D::getBFSCostToGoal(int x, int y, int z) const
{
  if(bfs_->getDistance(x,y,z) > 1000000)
    return INT_MAX;
  else
    return int(bfs_->getDistance(x,y,z)) * prm_->cost_per_cell_ * 1200 * grid_->getResolution();
}

int EnvironmentOMPLROBARM3D::getBFSCostToStart(int x, int y, int z) const
{
  if(bfs_b->getDistance(x,y,z) > 1000000)
    return INT_MAX;
  else
    return int(bfs_b->getDistance(x,y,z)) * prm_->cost_per_cell_;
}


int EnvironmentOMPLROBARM3D::getXYZHeuristic(int FromStateID, int ToStateID)
{
  EnvOMPLROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[FromStateID];

  //get distance heuristic
  if(prm_->use_bfs_heuristic_)
  {
    FromHashEntry->heur = getBFSCostToGoal(FromHashEntry->xyz[0], FromHashEntry->xyz[1], FromHashEntry->xyz[2]);
    
    // if(FromHashEntry->heur == INT_MAX)
    // {
    //   ROS_INFO("Heuristic state ID : %d is infinite", FromStateID);

    //   EnvOMPLROBARM3DHashEntry_t* hash_entry = pdata_.StateID2CoordTable[FromStateID];
    
    //   std::vector<int> coord(7, 0);
    //   std::vector<double> angles(7, 0);
    //   std::vector<double> pose(6, 0);

    //   for(int i = 0; i < prm_->num_joints_; i++)
    //     coord[i] = hash_entry->coord.at(i);
      
    //   coordToAngles(coord, angles);

    //   rmodel_->computePlanningLinkFK(angles, pose);

    //   std::vector<double> state(6,0);
    //   state[0] = pose[0];
    //   state[1] = pose[1];
    //   state[2] = pose[2];

    //   state[3] = pose[0];
    //   state[4] = pose[1];
    //   state[5] = pose[2];

    //   stateToVisualizationMarker(state, true, false, true);

    //   BodyPose body;
    //   body.x = 0;
    //   body.y = 0;
    //   body.z = 0.210000;
    //   body.theta = 0;
    //   vector<double> left_arm{ 1.500,  1.000,  0.006, -1.400,  0.001, -0.744,  0.000};

    //   pviz_.visualizeRobot(angles, left_arm, body, 120, "state", 2, true);

    //   getchar();
    // }

  } 
  else
  {
    double x, y, z;
    grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x, y, z);
    FromHashEntry->heur = getEuclideanDistance(x, y, z, pdata_.goal.pose[0], pdata_.goal.pose[1], pdata_.goal.pose[2]) * prm_->cost_per_meter_;
  }
  return FromHashEntry->heur;
}

int EnvironmentOMPLROBARM3D::getXYZHeuristicBack(int FromStateID, int ToStateID)
{
  EnvOMPLROBARM3DHashEntry_t* FromHashEntry = pdata_.StateID2CoordTable[FromStateID];

  //get distance heuristic
  if(prm_->use_bfs_heuristic_)
    FromHashEntry->heur = getBFSCostToStart(FromHashEntry->xyz[0], FromHashEntry->xyz[1], FromHashEntry->xyz[2]);
  else
  {
    double x, y, z;
    grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x, y, z);
    FromHashEntry->heur = getEuclideanDistance(x, y, z, pdata_.start.pose[0], pdata_.start.pose[1], pdata_.start.pose[2]) * prm_->cost_per_meter_;
  }
  return FromHashEntry->heur;
}
bool EnvironmentOMPLROBARM3D::convertStateIDPathToJointTrajectory(const std::vector<int> &idpath, trajectory_msgs::JointTrajectory &traj)
{
  if(idpath.empty())
    return false;

  traj.header.frame_id = prm_->planning_frame_;
  traj.joint_names = prm_->planning_joints_;
  traj.points.resize(idpath.size());
  
  std::vector<double> angles;
  for(size_t i = 0; i < idpath.size(); ++i)
  {
    traj.points[i].positions.resize(prm_->num_joints_);
    StateID2Angles(idpath[i], angles);

    for (int p = 0; p < prm_->num_joints_; ++p)
      traj.points[i].positions[p] = angles::normalize_angle(angles[p]); 
  }

  return true;
}

double EnvironmentOMPLROBARM3D::getDistanceToGoal(double x, double y, double z)
{
  double dist;
  int dx, dy, dz;
  grid_->worldToGrid(x, y, z, dx, dy, dz);

  if(prm_->use_bfs_heuristic_)
    dist = double(bfs_->getDistance(dx, dy, dz)) * grid_->getResolution();
  else
    dist = getEuclideanDistance(x, y, z, pdata_.goal.pose[0], pdata_.goal.pose[1], pdata_.goal.pose[2]);

  return dist;
}

double EnvironmentOMPLROBARM3D::getDistanceToStart(double x, double y, double z)
{
  double dist;
  int dx, dy, dz;
  grid_->worldToGrid(x, y, z, dx, dy, dz);

  if(prm_->use_bfs_heuristic_)
    dist = double(bfs_b->getDistance(dx, dy, dz)) * grid_->getResolution();
  else
    dist = getEuclideanDistance(x, y, z, pdata_.start.pose[0], pdata_.start.pose[1], pdata_.start.pose[2]);

  return dist;
}


double EnvironmentOMPLROBARM3D::getDistanceToFront(double x, double y, double z, bool forward)
{
  double dist;
  int dx, dy, dz;
  grid_->worldToGrid(x, y, z, dx, dy, dz);

  EnvOMPLROBARM3DHashEntry_t* FromHashEntry;
  if (forward)
    FromHashEntry = pdata_.StateID2CoordTable[state_id_last];
  else
    FromHashEntry = pdata_.StateID2CoordTable[state_id_last_b];
  double x_f, y_f, z_f;
  grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x_f, y_f, z_f);
  dist = getEuclideanDistance(x, y, z, x_f, y_f, z_f);

  return dist;
}

std::vector<double> EnvironmentOMPLROBARM3D::getGoal()
{
  return pdata_.goal.pose;
}

std::vector<double> EnvironmentOMPLROBARM3D::getStart()
{
  return pdata_.start.pose;
}

visualization_msgs::MarkerArray EnvironmentOMPLROBARM3D::getVisualization(std::string type)
{
  visualization_msgs::MarkerArray ma;

  if(type.compare("bfs_walls") == 0)
  {
    std::vector<geometry_msgs::Point> pnts;
    geometry_msgs::Point p;
    int dimX, dimY, dimZ;
    grid_->getGridSize(dimX, dimY, dimZ);
    for (int z = 0; z < dimZ - 2; z++)
      for (int y = 0; y < dimY - 2; y++)
        for (int x = 0; x < dimX - 2; x++)
          if(bfs_->isWall(x+1, y+1, z+1))
          {
            grid_->gridToWorld(x, y, z, p.x, p.y, p.z);
            pnts.push_back(p);
          }
    if(!pnts.empty())
      ma.markers.push_back(viz::getSpheresMarker(pnts, 0.02, 210, grid_->getReferenceFrame(), "bfs_walls", 0));
  }
  else if(type.compare("bfs_values") == 0)
  {
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;
    int dimX, dimY, dimZ;
    grid_->getGridSize(dimX, dimY, dimZ);
    for (int z = 65; z < 100 - 2; z++)
      for (int y = 50; y < 100 - 2; y++)
        for (int x = 65; x < 100 - 2; x++)
        {
          int d = bfs_->getDistance(x+1, y+1, z+1);
          if(d < 10000)
          {
            grid_->gridToWorld(x, y, z, p.position.x, p.position.y, p.position.z);
            double hue = d/30.0 * 300;
            ma.markers.push_back(viz::getTextMarker(p, boost::lexical_cast<std::string>(d), 0.009, hue, grid_->getReferenceFrame(), "bfs_values", ma.markers.size()));
          }
        }
  }
  else
    ROS_ERROR("No such marker type, '%s'.", type.c_str());
  
  bfs_pub.publish(ma);
  return ma;
}

//TODO(Karthik) Check Implementation
void EnvironmentOMPLROBARM3D::GetNearestLatticeState(const ompl::base::State *continuous_state, ompl::base::State* nearest_lattice_state, int *nearest_lattice_state_id) {
  
  //const ompl::base::RealVectorStateSpace::StateType* arm_state = continuous_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> state_angles(prm_->num_joints_,0);
  for(int i = 0; i < prm_->num_joints_; i++)
  {
    state_angles[i] = continuous_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    //if(state_angles[i] < 0)
    //  state_angles[i] += 2*M_PI; 
    //printf("the state angle at %d is %f\n", i , state_angles[i]);
  }

  std::vector<int> coord(prm_->num_joints_,0);
  anglesToCoord(state_angles, coord);

  for(int i = 0; i < prm_->num_joints_; i++)
  {
    //printf("the coord angle at %d is %d\n", i , coord[i]);
  }

  double dist = 0;
  int endeff[3]={0};

  //check if hash entry already exists, if not then create one
  bool succ_is_goal_state = false;
  std::vector<double> pose(6,0);

  // //get joint positions of closest configuration
  if(!rmodel_->computePlanningLinkFK(state_angles, pose))
    ROS_WARN("Unable to compute forward kinematics for the continous state. Attempting to plan anyway.");

  // //check if the closest configuration is in collision but plan anyway
  if(!cc_->isStateValid(state_angles, prm_->verbose_collisions_, false, dist))
  {
    //ROS_WARN("[env] The get nearest configuration is in collision. Attempting to plan anyway. (distance to nearest obstacle %0.2fm)", double(dist)*grid_->getResolution());
  }

  // if(isGoalState(pose, pdata_.goal))
  //   succ_is_goal_state = true;

  grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);

  EnvOMPLROBARM3DHashEntry_t* HashEntry;
  if((HashEntry = getHashEntry(coord, succ_is_goal_state)) == NULL)
  {
    HashEntry = createHashEntry(coord, endeff);
    HashEntry->dist = dist;
  }
  *nearest_lattice_state_id = HashEntry->stateID;

  std::vector<double> ret_angles(prm_->num_joints_,0);
  coordToAngles(coord, ret_angles);

  for(int i = 0; i < prm_->num_joints_; i++)
  {
    if(ret_angles[i] > M_PI)
      nearest_lattice_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = ret_angles[i] - 2*M_PI;
    else
      nearest_lattice_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = ret_angles[i];
    //printf("the ret angle at %d is %f\n", i , ret_angles[i]);
  }

}

//TODO(Karthik) Check Implementation
void EnvironmentOMPLROBARM3D::VisualizeContState(const ompl::base::State *state, const ompl::base::State *parent_state, bool is_discrete, bool is_path) {
  //return;

  const ompl::base::RealVectorStateSpace::StateType* child = state->as<ompl::base::RealVectorStateSpace::StateType>();
  
  std::vector<double> child_angles(prm_->num_joints_, 0);
  std::vector<double> child_pose(prm_->num_joints_, 0);
  for(size_t i=0; i < prm_->num_joints_; i++)
  {  
     if(child->values[i] < 0)
      child_angles[i] = child->values[i] + 2*M_PI;
     else
      child_angles[i] = child->values[i];
  }
  rmodel_->computePlanningLinkFK(child_angles, child_pose);

  const ompl::base::RealVectorStateSpace::StateType* parent = parent_state->as<ompl::base::RealVectorStateSpace::StateType>();
  
  std::vector<double> parent_angles(prm_->num_joints_, 0);
  std::vector<double> parent_pose(prm_->num_joints_, 0);
  for(size_t i=0; i < prm_->num_joints_; i++)
  {  
     if(parent->values[i] < 0)
      parent_angles[i] = parent->values[i] + 2*M_PI;
     else
      parent_angles[i] = parent->values[i];
  }
  rmodel_->computePlanningLinkFK(parent_angles, parent_pose);

  //ROS_INFO("[Visualization] Child state in angles is %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f", child_angles[0], child_angles[1], child_angles[2], child_angles[3],
  //                                                                                                      child_angles[4], child_angles[5], child_angles[6]);
  //ROS_INFO("[Visualization] Parent state in angles is %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f", parent_angles[0],  parent_angles[1],  parent_angles[2],  parent_angles[3],
  //                                                                                                         parent_angles[4],  parent_angles[5], parent_angles[6]);
  std::vector<double> coord(6,0);
  coord[0] = child_pose[0];
  coord[1] = child_pose[1];
  coord[2] = child_pose[2];
  coord[3] = parent_pose[0];
  coord[4] = parent_pose[1];
  coord[5] = parent_pose[2];

  stateToVisualizationMarker(coord, is_discrete, is_path, use_visualization_);
}

//TODO(Karthik) Check Implementation
// Get the continuous-state equivalent for a lattice state.
void EnvironmentOMPLROBARM3D::GetContState(int state_id, ompl::base::State *state) {
  EnvOMPLROBARM3DHashEntry_t* HashEntry = pdata_.StateID2CoordTable[state_id];

  std::vector<int> coord(prm_->num_joints_,0);
  std::vector<double> angles(prm_->num_joints_,0);
  std::vector<double> pose(6,0);

  for(int i = 0; i < prm_->num_joints_; i++)
  {
    coord[i] = HashEntry->coord.at(i);
    //printf("Coord at %d is %d\n", i, coord[i]);
  }

  coordToAngles(coord, angles);
  for(int i = 0; i < prm_->num_joints_; i++)
  {
    //printf("Angle at %d is %f\n", i, angles[i]);
  }

  for(int i = 0; i < prm_->num_joints_; i++)
  {
    if(angles[i] > M_PI)
    {
      state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = angles[i] - 2*M_PI;
    }
    else
    {
      state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = angles[i];
    }

  }

  //rmodel_->computePlanningLinkFK(dummy_angles, pose);
  //printf("From get cont state pose is %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\n", pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);

}

//TODO(Karthik) Check Implementation
int EnvironmentOMPLROBARM3D::GetContStateID(const ompl::base::State* state) {

  int cont_state_id = cont_hash_counter;
  cont_hash_counter++;
  if (cont_hash_counter < SizeofCreatedEnv()) {
    printf("We have a ID problem!!!!!!!!\n");
    exit(1);
  }

  //printf("The continous StateID is %d\n", cont_state_id);fflush(stdout);

  return cont_state_id;
}

//TODO(Karthik) Check Implementation
int EnvironmentOMPLROBARM3D::GetContEdgeCost(const ompl::base::State *parent_state, const ompl::base::State *child_state) {
  // TODO: this should use the same optimization objective used by the OMPL
  // benchmarks. Also, GetActionCost should call this method to compute the cost
  // of an edge.
  // printf("Hi..................................\n");fflush(stdout);

  std::vector<int> parent_coord(prm_->num_joints_,0);
  std::vector<double> parent_angles(prm_->num_joints_,0);
  for(int i = 0; i < 7; i++)
  {
    parent_angles[i] = parent_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];

    //printf("the parent angle at %d is %f\n", i , parent_angles[i]);
  }

  std::vector<int> child_coord(prm_->num_joints_,0);
  std::vector<double> child_angles(prm_->num_joints_,0);
  for(int i = 0; i < 7; i++)
  {
    child_angles[i] = child_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];

    //printf("the child angle at %d is %f\n", i , child_angles[i]);
  }

  std::vector<std::pair<double, double>> angle_limits(7);
  angle_limits.push_back(std::make_pair(-2.135398, 0.715));
  angle_limits.push_back(std::make_pair(-0.355, 1.2963));
  angle_limits.push_back(std::make_pair(-3.75, 0.865));
  angle_limits.push_back(std::make_pair(-2.1213, 0.150));
  angle_limits.push_back(std::make_pair(-M_PI, M_PI));
  angle_limits.push_back(std::make_pair(-2.0, 0.0));
  angle_limits.push_back(std::make_pair(-M_PI, M_PI));

  std::vector<int> diff_coord(prm_->num_joints_,0);
  std::vector<double> diff_angles(prm_->num_joints_,0);
  for(int i = 0; i < 7; i++){
    //angles::shortest_angular_distance_with_limits(parent_angles[i],child_angles[i],angle_limits[i].first,angle_limits[i].second,diff_angles[i]); 
    diff_angles[i] = angles::shortest_angular_distance(parent_angles[i],child_angles[i]); 
  }

  //double max_diff = 0.0;

  //for(size_t i = 0; i < 7; i++)
  //{
  //  if(max_diff < diff_angles[i])
  //    max_diff = diff_angles[i];
  //}

  //max_diff = max_diff*180/M_PI;

  //printf("the max diff is given by %0.3f\n", max_diff);

  //int cost =  (1000/4)*(max_diff);

  double euclidean_diff = sqrt(diff_angles[0]*diff_angles[0] + diff_angles[1]*diff_angles[1] + diff_angles[2]*diff_angles[2] +
                               diff_angles[3]*diff_angles[3] + diff_angles[4]*diff_angles[4] + diff_angles[5]*diff_angles[5] +
                               diff_angles[6]*diff_angles[6] );



  int cost = 1000*(euclidean_diff);

  return cost;

}

void EnvironmentOMPLROBARM3D::stateToVisualizationMarker(std::vector<double> coord, bool is_discrete_state, bool is_path, bool check){
  if (!check) return;

  visualization_msgs::Marker state;
  state.header.frame_id = "base_footprint";
  state.header.stamp = ros::Time();
  state.id = rand();
  state.action = visualization_msgs::Marker::ADD;
  state.type = visualization_msgs::Marker::CUBE;
  state.scale.x = 0.005;
  state.scale.y = 0.005;
  state.scale.z = 0.005;
  state.pose.position.x = coord[0];
  state.pose.position.y = coord[1];
  state.pose.position.z = coord[2];
  state.pose.orientation.w = 1.0;

  visualization_msgs::Marker line_strip;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.id = rand();
  line_strip.scale.x = 0.005;
  line_strip.scale.y = 0.005;
  line_strip.scale.z = 0.005;
  line_strip.header.frame_id = "base_footprint";
  line_strip.lifetime = ros::Duration();

  visualization_msgs::Marker path_strip;
  path_strip.type = visualization_msgs::Marker::LINE_STRIP;
  path_strip.id = rand();
  path_strip.scale.x = 0.005;
  path_strip.scale.y = 0.005;
  path_strip.scale.z = 0.005;
  path_strip.header.frame_id = "base_footprint";
  path_strip.lifetime = ros::Duration();

  if (is_discrete_state && !is_path){
  state.color.r = 1.0f;
  state.color.g = 0.0f;
  state.color.b = 0.0f;
  state.color.a = 1.0;

  geometry_msgs::Point point;
  point.x = coord[0];
  point.y = coord[1];
  point.z = coord[2];
  line_strip.points.push_back(point);
  point.x = coord[3];
  point.y = coord[4];
  point.z = coord[5];
  line_strip.points.push_back(point);
  line_strip.color.r = 1.0f;
  line_strip.color.a = 1.0;

  tree_pub.publish(line_strip);  
  } 
  else if (!is_discrete_state && !is_path){
  state.color.r = 0.0f;
  state.color.g = 1.0f;
  state.color.b = 0.0f;
  state.color.a = 1.0;

  geometry_msgs::Point point;
  point.x = coord[0];
  point.y = coord[1];
  point.z = coord[2];;
  line_strip.points.push_back(point);
  
  point.x = coord[3];
  point.y = coord[4];
  point.z = coord[5];
  line_strip.points.push_back(point);

  line_strip.color.g = 1.0f;
  line_strip.color.a = 1.0;
  tree_pub.publish(line_strip);
  }
  else{
  state.color.r = 0.0f;
  state.color.g = 0.0f;
  state.color.b = 0.0f;
  state.color.a = 1.0;

  geometry_msgs::Point point;
  point.x = coord[0];
  point.y = coord[1];
  point.z = coord[2];
  path_strip.points.push_back(point);

  point.x = coord[3];
  point.y = coord[4];
  point.z = coord[5];
  path_strip.points.push_back(point);

  path_strip.color.r = 0.0f;
  path_strip.color.g = 0.0f;
  path_strip.color.b = 0.0f;
  path_strip.color.a = 1.0;
  tree_pub.publish(path_strip);
  }
  state.lifetime = ros::Duration();
  tree_pub.publish(state);
  if(slow_visualize)
    ros::Duration(0.1).sleep();
}

void EnvironmentOMPLROBARM3D::printContState(std::vector<double> angles){

  std::vector<double> pose(6,0);
  printf("Cont state angles is %f, %f, %f, %f, %f, %f, %f\n", angles[0], angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);

  rmodel_->computePlanningLinkFK(angles, pose);
  printf("Cont state pose is %f, %f, %f, %f, %f, %f\n", pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);
}

}

