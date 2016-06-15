/*
 * Copyright (c) 2015, Maxim Likhachev
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
 *     * Neither the name of the Carnegie Mellon University nor the names of its
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
#include <ctime>
#include <sbpl/discrete_space_information/environment_ompl.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

#include "/usr/local/include/ompl/base/spaces/SE2StateSpace.h"
#include "/usr/local/include/ompl/base/OptimizationObjective.h"
#include "/usr/local/include/ompl/base/Cost.h"

#include <angles/angles.h>


using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0;


#define XYTHETA2INDEX(X,Y,THETA) (THETA + X*EnvOMPLCfg.NumThetaDirs + \
                                  Y*EnvOMPLCfg.EnvWidth_c*EnvOMPLCfg.NumThetaDirs)

//cell size
const double kCellSize =  0.1;//0.4296875;//

std::string point_rob_name = "point_robot_tall_10mm.dae";

std::vector<double> coord_disc;

//-----------------constructors/destructors-------------------------------

EnvironmentOMPLTICE::EnvironmentOMPLTICE(const ompl::base::SpaceInformationPtr &si)
{
    EnvOMPLCfg.obsthresh = ENVOMPL_DEFAULTOBSTHRESH;
    //the value that pretty much makes it disabled
    EnvOMPLCfg.cost_inscribed_thresh = EnvOMPLCfg.obsthresh; 
    //the value that pretty much makes it disabled
    EnvOMPLCfg.cost_possibly_circumscribed_thresh = -1; 

    grid2Dsearchfromstart = NULL;
    grid2Dsearchfromgoal = NULL;
    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;
    iteration = 0;
    bucketsize = 0; // fixed bucket size
    blocksize = 1;

    EnvOMPL.bInitialized = false;

    EnvOMPLCfg.actionwidth = OMPL_DEFAULT_ACTIONWIDTH;

    EnvOMPLCfg.NumThetaDirs = OMPL_THETADIRS;

    //no memory allocated in cfg yet
    EnvOMPLCfg.Grid2D = NULL;
    EnvOMPLCfg.ActionsV = NULL;
    EnvOMPLCfg.PredActionsV = NULL;

    //Succesors state visualisation publishers
    vis_pub = nh.advertise<visualization_msgs::Marker>("state_marker", 1000);
    line_pub = nh.advertise<visualization_msgs::Marker>("line_marker", 1000);
    env_pub = nh.advertise<visualization_msgs::Marker>("env_marker", 1000);
    rob_start_pub = nh.advertise<visualization_msgs::Marker>("rob_s_marker", 1000);
    rob_goal_pub = nh.advertise<visualization_msgs::Marker>("rob_g_marker", 1000);
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1000);

    //Environment Path
    env_path = "../env_examples/ompl/Maze_planar/";
    package_path = "sbpl/env_examples/ompl/Maze_planar/";
    file_cont = "package://";

    //Continous state hash counter
    cont_hash_counter = SBPL_OMPL_MAXSTATESFORLOOKUP;

    //Get space information
    si_ = si;

    ros::NodeHandle private_nh("~");
    private_nh.param("visualize_h_star", use_visualization_, false);
}

EnvironmentOMPLTICE::~EnvironmentOMPLTICE()
{
    SBPL_PRINTF("destroying OMPLTICE\n");
    if (grid2Dsearchfromstart != NULL) delete grid2Dsearchfromstart;
    grid2Dsearchfromstart = NULL;

    if (grid2Dsearchfromgoal != NULL) delete grid2Dsearchfromgoal;
    grid2Dsearchfromgoal = NULL;

    if (EnvOMPLCfg.Grid2D != NULL) {
        for (int x = 0; x < EnvOMPLCfg.EnvWidth_c; x++)
            delete[] EnvOMPLCfg.Grid2D[x];
        delete[] EnvOMPLCfg.Grid2D;
        EnvOMPLCfg.Grid2D = NULL;
    }

    //delete actions
    if (EnvOMPLCfg.ActionsV != NULL) {
        for (int tind = 0; tind < EnvOMPLCfg.NumThetaDirs; tind++)
            delete[] EnvOMPLCfg.ActionsV[tind];
        delete[] EnvOMPLCfg.ActionsV;
        EnvOMPLCfg.ActionsV = NULL;
    }
    if (EnvOMPLCfg.PredActionsV != NULL) {
        delete[] EnvOMPLCfg.PredActionsV;
        EnvOMPLCfg.PredActionsV = NULL;
    }
}

//---------------------------------------------------------------------

//------------------Environment and robot publisher---------------------------

void EnvironmentOMPLTICE::publish_env_rob(){
  if (!use_visualization_) return;
  visualization_msgs::Marker env;
  env.header.frame_id = "map";
  env.header.stamp = ros::Time();
  env.id = rand();
  env.action = visualization_msgs::Marker::ADD;
  env.type = visualization_msgs::Marker::MESH_RESOURCE;
  env.scale.x = 39.3700787402;
  env.scale.y = 39.3700787402;
  env.scale.z = 39.3700787402;
  env.pose.position.x = 0;
  env.pose.position.y = 0;
  env.pose.position.z = 0;
  env.pose.orientation.x = -0.7071067811865475;
  env.pose.orientation.y = 0;
  env.pose.orientation.z = 0;
  env.pose.orientation.w = 0.7071067811865476;
  env.color.r = 0.0f;
  env.color.g = 1.0f;
  env.color.b = 1.0f;
  env.color.a = 1.0;
  env.lifetime = ros::Duration();
  env.mesh_resource =  file_cont + package_path + EnvOMPLCfg.env_name;
  ma.markers.push_back(env); 

  visualization_msgs::Marker rob_start;
  rob_start.header.frame_id = "map";
  rob_start.header.stamp = ros::Time();
  rob_start.id = rand();
  rob_start.action = visualization_msgs::Marker::ADD;
  rob_start.type = visualization_msgs::Marker::MESH_RESOURCE;
  rob_start.scale.x = 39.3700787402;
  rob_start.scale.y = 39.3700787402;
  rob_start.scale.z = 39.3700787402;
  rob_start.pose.position.x = EnvOMPLCfg.start_x;
  rob_start.pose.position.y = EnvOMPLCfg.start_y;
  rob_start.pose.position.z = 0;
  rob_start.pose.orientation.x = -0.7071067811865475;
  rob_start.pose.orientation.y = 0;
  rob_start.pose.orientation.z = 0;
  rob_start.pose.orientation.w = 0.7071067811865476;
  rob_start.color.r = 1.0f;
  rob_start.color.g = 1.0f;
  rob_start.color.b = 0.0f;
  rob_start.color.a = 1.0;
  rob_start.lifetime = ros::Duration();
  rob_start.mesh_resource = file_cont + package_path + EnvOMPLCfg.robot_name;
  ma.markers.push_back(rob_start);

  visualization_msgs::Marker rob_goal;
  rob_goal.header.frame_id = "map";
  rob_goal.header.stamp = ros::Time();
  rob_goal.id = rand();
  rob_goal.action = visualization_msgs::Marker::ADD;
  rob_goal.type = visualization_msgs::Marker::MESH_RESOURCE;
  rob_goal.scale.x = 39.3700787402;
  rob_goal.scale.y = 39.3700787402;
  rob_goal.scale.z = 39.3700787402;
  rob_goal.pose.position.x = EnvOMPLCfg.goal_x;
  rob_goal.pose.position.y = EnvOMPLCfg.goal_y;
  rob_goal.pose.position.z = 0;
  rob_goal.pose.orientation.x = -0.7071067811865475;
  rob_goal.pose.orientation.y = 0;
  rob_goal.pose.orientation.z = 0;
  rob_goal.pose.orientation.w = 0.7071067811865476;
  rob_goal.color.r = 1.0f;
  rob_goal.color.g = 1.0f;
  rob_goal.color.b = 0.0f;
  rob_goal.color.a = 1.0;
  rob_goal.lifetime = ros::Duration();
  rob_goal.mesh_resource = file_cont + package_path + EnvOMPLCfg.robot_name;
  ma.markers.push_back(rob_goal);
}

//-------------------problem specific and local functions---------------------

static unsigned int inthash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

void EnvironmentOMPLTICE::SetConfiguration(int width, int height, const unsigned char* mapdata, int startx,
                                                    int starty, int starttheta, int goalx, int goaly, int goaltheta,
                                                    double cellsize_m, double nominalvel_mpersecs,
                                                    double timetoturn45degsinplace_secs,
                                                    const vector<sbpl_2Dpt_t> & robot_perimeterV)
{
    EnvOMPLCfg.EnvWidth_c = width;
    EnvOMPLCfg.EnvHeight_c = height;
    EnvOMPLCfg.StartX_c = startx;
    EnvOMPLCfg.StartY_c = starty;
    EnvOMPLCfg.StartTheta = starttheta;

    if (EnvOMPLCfg.StartX_c < 0 || EnvOMPLCfg.StartX_c >= EnvOMPLCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvOMPLCfg.StartY_c < 0 || EnvOMPLCfg.StartY_c >= EnvOMPLCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvOMPLCfg.StartTheta < 0 || EnvOMPLCfg.StartTheta >= EnvOMPLCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
        throw new SBPL_Exception();
    }

    EnvOMPLCfg.EndX_c = goalx;
    EnvOMPLCfg.EndY_c = goaly;
    EnvOMPLCfg.EndTheta = goaltheta;

    if (EnvOMPLCfg.EndX_c < 0 || EnvOMPLCfg.EndX_c >= EnvOMPLCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal goal coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvOMPLCfg.EndY_c < 0 || EnvOMPLCfg.EndY_c >= EnvOMPLCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal goal coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvOMPLCfg.EndTheta < 0 || EnvOMPLCfg.EndTheta >= EnvOMPLCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
        throw new SBPL_Exception();
    }

    EnvOMPLCfg.FootprintPolygon = robot_perimeterV;

    EnvOMPLCfg.nominalvel_mpersecs = nominalvel_mpersecs;
    EnvOMPLCfg.cellsize_m = cellsize_m;
    EnvOMPLCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;

    //allocate the 2D environment
    EnvOMPLCfg.Grid2D = new unsigned char*[EnvOMPLCfg.EnvWidth_c];
    for (int x = 0; x < EnvOMPLCfg.EnvWidth_c; x++) {
        EnvOMPLCfg.Grid2D[x] = new unsigned char[EnvOMPLCfg.EnvHeight_c];
    }

    //environment:
    if (0 == mapdata) {
        for (int y = 0; y < EnvOMPLCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvOMPLCfg.EnvWidth_c; x++) {
                EnvOMPLCfg.Grid2D[x][y] = 0;
            }
        }
    }
    else {
        for (int y = 0; y < EnvOMPLCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvOMPLCfg.EnvWidth_c; x++) {
                EnvOMPLCfg.Grid2D[x][y] = mapdata[x + y * width];
            }
        }
    }
} 

void EnvironmentOMPLTICE::ReadConfiguration(const char* fCfg)
{
    //read in the configuration of environment and initialize  EnvOMPLCfg structure
    
    ConfigFile cf(fCfg);
    int x, y;

    //Read problem name, robot and environment mesh
    std::string name = cf.Value("problem","name");
    EnvOMPLCfg.name = name;

    std::string robot_name = cf.Value("problem","robot");
    EnvOMPLCfg.robot_name = robot_name;

    std::string env_name = cf.Value("problem","world");
    EnvOMPLCfg.env_name = env_name;

    //Define start state
    EnvOMPLCfg.start_x = cf.Value("problem","start.x");
    EnvOMPLCfg.start_y = cf.Value("problem","start.y");
    EnvOMPLCfg.start_theta = cf.Value("problem","start.theta");

    //Define goal state
    EnvOMPLCfg.goal_x = cf.Value("problem","goal.x");
    EnvOMPLCfg.goal_y = cf.Value("problem","goal.y");
    EnvOMPLCfg.goal_theta = cf.Value("problem","goal.theta");

    //environment bounds
    EnvOMPLCfg.volume_min_x = cf.Value("problem","volume.min.x");
    EnvOMPLCfg.volume_min_y = cf.Value("problem","volume.min.y");
    EnvOMPLCfg.volume_max_x = cf.Value("problem","volume.max.x");
    EnvOMPLCfg.volume_max_y = cf.Value("problem","volume.max.y");

    // Scan for optional NumThetaDirs parameter
    EnvOMPLCfg.NumThetaDirs = OMPL_THETADIRS;

    // obsthresh
    EnvOMPLCfg.obsthresh = 1;

    //cost_inscribed_thresh: 
    EnvOMPLCfg.cost_inscribed_thresh = 1;
  
    //cost_possibly_circumscribed_thresh: 
    EnvOMPLCfg.cost_possibly_circumscribed_thresh = 0;

    //number of cells
    EnvOMPLCfg.EnvWidth_c = (int)( (EnvOMPLCfg.volume_max_x - EnvOMPLCfg.volume_min_x) / kCellSize);
    EnvOMPLCfg.EnvHeight_c = (int)( (EnvOMPLCfg.volume_max_y - EnvOMPLCfg.volume_min_y) / kCellSize);

    //cellsize
    EnvOMPLCfg.cellsize_m = kCellSize;

    //speeds
    EnvOMPLCfg.nominalvel_mpersecs = 1.0;
    EnvOMPLCfg.timetoturn45degsinplace_secs = 2.0;

    //start(meters,rads):
    EnvOMPLCfg.StartX_c = CONTXY2DISC(EnvOMPLCfg.start_x - EnvOMPLCfg.volume_min_x, EnvOMPLCfg.cellsize_m);
    EnvOMPLCfg.StartY_c = CONTXY2DISC(EnvOMPLCfg.start_y - EnvOMPLCfg.volume_min_y, EnvOMPLCfg.cellsize_m);
    EnvOMPLCfg.StartTheta = ContTheta2Disc(normalizeAngle(EnvOMPLCfg.start_theta), EnvOMPLCfg.NumThetaDirs);

    if (EnvOMPLCfg.StartX_c < 0 || EnvOMPLCfg.StartX_c >= EnvOMPLCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvOMPLCfg.StartY_c < 0 || EnvOMPLCfg.StartY_c >= EnvOMPLCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal start coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvOMPLCfg.StartTheta < 0 || EnvOMPLCfg.StartTheta >= EnvOMPLCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
        throw new SBPL_Exception();
    }

    //printf("Start: %d %d %d\n", EnvOMPLCfg.StartX_c, EnvOMPLCfg.StartY_c, EnvOMPLCfg.StartTheta);

    //end(meters,rads):
    EnvOMPLCfg.EndX_c = CONTXY2DISC(EnvOMPLCfg.goal_x - EnvOMPLCfg.volume_min_x, EnvOMPLCfg.cellsize_m);
    EnvOMPLCfg.EndY_c = CONTXY2DISC(EnvOMPLCfg.goal_y - EnvOMPLCfg.volume_min_y, EnvOMPLCfg.cellsize_m);
    EnvOMPLCfg.EndTheta = ContTheta2Disc(normalizeAngle(EnvOMPLCfg.goal_theta), EnvOMPLCfg.NumThetaDirs);

    if (EnvOMPLCfg.EndX_c < 0 || EnvOMPLCfg.EndX_c >= EnvOMPLCfg.EnvWidth_c) {
        SBPL_ERROR("ERROR: illegal end coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvOMPLCfg.EndY_c < 0 || EnvOMPLCfg.EndY_c >= EnvOMPLCfg.EnvHeight_c) {
        SBPL_ERROR("ERROR: illegal end coordinates\n");
        throw new SBPL_Exception();
    }
    if (EnvOMPLCfg.EndTheta < 0 || EnvOMPLCfg.EndTheta >= EnvOMPLCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
        throw new SBPL_Exception();
    }
    //printf("End: %d %d %d\n", EnvOMPLCfg.EndX_c, EnvOMPLCfg.EndY_c, EnvOMPLCfg.EndTheta);

    //allocate the 2D environment
    EnvOMPLCfg.Grid2D = new unsigned char*[EnvOMPLCfg.EnvWidth_c];
    for (x = 0; x < EnvOMPLCfg.EnvWidth_c; x++) {
        EnvOMPLCfg.Grid2D[x] = new unsigned char[EnvOMPLCfg.EnvHeight_c];
    }

    //The stored grid map input file name
    std::string this_name = EnvOMPLCfg.env_name;
    std::string txt_name = this_name.erase(this_name.length() - 4);
    std::string txt_ext = ".txt";
    std::string intxt_file = env_path + txt_name + txt_ext;   

    if(!fopen(intxt_file.c_str(), "r")){
        //The grid map out file name
        std::ofstream out(intxt_file.c_str());

        //the point robot path
        std::string in_prob_file = env_path + point_rob_name;

        //the point env path
        std::string in_env_file = env_path + EnvOMPLCfg.env_name;

        //get the projected collision map
        std::vector<std::vector<int> > v = collision_map_project(in_prob_file.c_str(),in_env_file.c_str());  
        
        //write to .txt file
        for (y = 0; y < EnvOMPLCfg.EnvHeight_c; y++){
            for (x = 0; x < EnvOMPLCfg.EnvWidth_c; x++) {
                EnvOMPLCfg.Grid2D[x][y] = v[x][y];
                out << v[x][y] << " ";
            }
            out << "\n";
        }
    }
    else{
        std::vector<std::vector<int> > v = get_map_file(intxt_file.c_str());

        //environment:
        for (y = 0; y < EnvOMPLCfg.EnvHeight_c; y++)
            for (x = 0; x < EnvOMPLCfg.EnvWidth_c; x++) {
                EnvOMPLCfg.Grid2D[x][y] = v[y][x];
            }

    }

}   

std::vector<std::vector<int> > EnvironmentOMPLTICE::collision_map_project(const char *prob, const char *env)
{   
    ompl::app::SE2RigidBodyPlanning fake_setup;
    fake_setup.setEnvironmentMesh(env);
    fake_setup.setRobotMesh(prob);

    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(fake_setup.getStateSpace());
    start->setX(EnvOMPLCfg.start_x);
    start->setY(EnvOMPLCfg.start_y);
    start->setYaw(EnvOMPLCfg.start_theta);

    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(fake_setup.getStateSpace());
    goal->setX(EnvOMPLCfg.goal_x);
    goal->setY(EnvOMPLCfg.goal_y);
    goal->setYaw(EnvOMPLCfg.goal_theta);
    
    fake_setup.setStartAndGoalStates(start, goal, .1);
    fake_setup.setup();
    
    ompl::base::State *state = fake_setup.getStateSpace()->allocState();
    double ompl_cont_x, ompl_cont_y, ompl_cont_theta;
    std::vector<std::vector<int> > v(EnvOMPLCfg.EnvWidth_c, std::vector<int>(EnvOMPLCfg.EnvHeight_c));

    for (int y = 0; y < EnvOMPLCfg.EnvHeight_c; y+=1){
        for (int x = 0; x < EnvOMPLCfg.EnvWidth_c; x+=1) {
            ompl_cont_x = DISCXY2CONT(x, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_x;
            ompl_cont_y = DISCXY2CONT(y, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_y;
            ompl_cont_theta = 0.0;

            state->as<ompl::base::SE2StateSpace::StateType>()->setX(ompl_cont_x);
            state->as<ompl::base::SE2StateSpace::StateType>()->setY(ompl_cont_y);
            state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(angles::normalize_angle(ompl_cont_theta));
            v[x][y] = 0;
        
            //check validity for map
            if(!(fake_setup.getStateValidityChecker()->isValid(state))){
                v[x][y] = 1;
            }
        }
    }

    fake_setup.getStateSpace()->freeState(state);

    return v;
}

std::vector<std::vector<int> >  EnvironmentOMPLTICE::get_map_file(const char *file)
{
    std::fstream in(file);
    std::string line;
    std::vector<std::vector<int> > v;
    int i = 0;

    while (std::getline(in, line))
    {
        int value;
        std::stringstream ss(line);

        v.push_back(std::vector<int>());  

        while (ss >> value)
        {
            v[i].push_back(value);
        }
        ++i;
    }

    return v;
}

std::vector<std::vector<int> > EnvironmentOMPLTICE::map_project(const char *argv)
{   
    int y_start = 0;
    int count = 0;
    int x = 0;

    std::vector<std::vector<int> > v(OMPL_NUM_CELL, std::vector<int>(OMPL_NUM_CELL*OMPL_NUM_CELL));
    std::vector<std::vector<int> > v_proj(OMPL_NUM_CELL, std::vector<int>(OMPL_NUM_CELL));

    v = get_map_file(argv);

    while(x < OMPL_NUM_CELL){
        for(int y = y_start; y < (y_start+OMPL_NUM_CELL); y++){
            if(v[y][x] == 1){
                v_proj[count][x] = 1;
                break;;
            }
            v_proj[count][x] = 0;
        }
        count = count + 1;
        y_start = y_start + OMPL_NUM_CELL;
        if(y_start > (OMPL_NUM_CELL*OMPL_NUM_CELL - 1)){
         x = x + 1; 
         y_start = 0;
         count = 0;
        }
    }

    return v_proj;
}

bool EnvironmentOMPLTICE::ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->x = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->y = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->theta = atoi(sTemp);

    //normalize the angle
    cell->theta = NORMALIZEDISCTHETA(cell->theta, EnvOMPLCfg.NumThetaDirs);

    return true;
}

bool EnvironmentOMPLTICE::ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->x = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->y = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->theta = atof(sTemp);

    pose->theta = normalizeAngle(pose->theta);

    return true;
}

bool EnvironmentOMPLTICE::ReadinMotionPrimitive(SBPL_ompl_mprimitive* pMotPrim, FILE* fIn)
{
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;

    //read in actionID
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1) return false;

    //read in start angle
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0) {
        SBPL_ERROR("ERROR reading startangle\n");
        return false;
    }
    pMotPrim->starttheta_c = dTemp;

    //read in end pose
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }

    if (ReadinCell(&pMotPrim->endcell, fIn) == false) {
        SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
        return false;
    }

    //read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1) return false;
    pMotPrim->additionalactioncostmult = dTemp;

    //read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1) return false;
    //all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done
    //after the action is rotated by initial orientation
    for (int i = 0; i < numofIntermPoses; i++) {
        sbpl_xy_theta_pt_t intermpose;
        if (ReadinPose(&intermpose, fIn) == false) {
            SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
            return false;
        }
        pMotPrim->intermptV.push_back(intermpose);
    }

    //check that the last pose corresponds correctly to the last pose
    sbpl_xy_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, EnvOMPLCfg.cellsize_m);
    sourcepose.y = DISCXY2CONT(0, EnvOMPLCfg.cellsize_m);
    sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, EnvOMPLCfg.NumThetaDirs);
    double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;
    int endx_c = CONTXY2DISC(mp_endx_m, EnvOMPLCfg.cellsize_m);
    int endy_c = CONTXY2DISC(mp_endy_m, EnvOMPLCfg.cellsize_m);
    int endtheta_c = ContTheta2Disc(mp_endtheta_rad, EnvOMPLCfg.NumThetaDirs);
    if (endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta) {
        SBPL_ERROR( "ERROR: incorrect primitive %d with startangle=%d "
                   "last interm point %f %f %f does not match end pose %d %d %d\n",
                   pMotPrim->motprimID, pMotPrim->starttheta_c,
                   pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x,
                   pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y,
                   pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta,
                   pMotPrim->endcell.x, pMotPrim->endcell.y,
                   pMotPrim->endcell.theta);
        return false;
    }

    return true;
}

bool EnvironmentOMPLTICE::ReadMotionPrimitives(FILE* fMotPrims)
{
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    SBPL_PRINTF("Reading in motion primitives...");

    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) return false;
    if (fabs(fTemp - EnvOMPLCfg.cellsize_m) > 0.1) {
        SBPL_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n", fTemp,
                   EnvOMPLCfg.cellsize_m);
        return false;
    }

    //read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0) return false;
    if (dTemp != EnvOMPLCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n",
                   dTemp, EnvOMPLCfg.NumThetaDirs);
        return false;
    }

    //read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
        return false;
    }

    for (int i = 0; i < totalNumofActions; i++) {
        SBPL_ompl_mprimitive motprim;

        if (EnvironmentOMPLTICE::ReadinMotionPrimitive(&motprim, fMotPrims) == false) return false;

        EnvOMPLCfg.mprimV.push_back(motprim);
    }
    SBPL_PRINTF("done ");

    return true;
}

void EnvironmentOMPLTICE::ComputeReplanningDataforAction(EnvOMPLAction_t* action)
{
    int j;

    //iterate over all the cells involved in the action
    sbpl_xy_theta_cell_t startcell3d, endcell3d;
    for (int i = 0; i < (int)action->intersectingcellsV.size(); i++) {
        //compute the translated affected search Pose - what state has an
        //outgoing action whose intersecting cell is at 0,0
        startcell3d.theta = action->starttheta;
        startcell3d.x = -action->intersectingcellsV.at(i).x;
        startcell3d.y = -action->intersectingcellsV.at(i).y;

        //compute the translated affected search Pose - what state has an
        //incoming action whose intersecting cell is at 0,0
        endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvOMPLCfg.NumThetaDirs);
        endcell3d.x = startcell3d.x + action->dX;
        endcell3d.y = startcell3d.y + action->dY;

        //store the cells if not already there
        for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
            if (affectedsuccstatesV.at(j) == endcell3d) break;
        }
        if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

        for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
            if (affectedpredstatesV.at(j) == startcell3d) break;
        }
        if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);
    }//over intersecting cells

    //add the centers since with h2d we are using these in cost computations
    //---intersecting cell = origin
    //compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -0;
    startcell3d.y = -0;

    //compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
    endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvOMPLCfg.NumThetaDirs);
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    //store the cells if not already there
    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) break;
    }
    if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) break;
    }
    if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);

    //---intersecting cell = outcome state
    //compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -action->dX;
    startcell3d.y = -action->dY;

    //compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
    endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvOMPLCfg.NumThetaDirs);
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) break;
    }
    if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) break;
    }
    if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);
}

//computes all the 3D states whose outgoing actions are potentially affected
//when cell (0,0) changes its status it also does the same for the 3D states
//whose incoming actions are potentially affected when cell (0,0) changes its
//status
void EnvironmentOMPLTICE::ComputeReplanningData()
{
    //iterate over all actions
    //orientations
    for (int tind = 0; tind < EnvOMPLCfg.NumThetaDirs; tind++) {
        //actions
        for (int aind = 0; aind < EnvOMPLCfg.actionwidth; aind++) {
            //compute replanning data for this action 
            ComputeReplanningDataforAction(&EnvOMPLCfg.ActionsV[tind][aind]);
        }
    }
}

//here motionprimitivevector contains actions only for 0 angle
void EnvironmentOMPLTICE::PrecomputeActionswithBaseMotionPrimitive(
        vector<SBPL_ompl_mprimitive>* motionprimitiveV)
{
    SBPL_PRINTF("Pre-computing action data using base motion primitives...\n");
    EnvOMPLCfg.ActionsV = new EnvOMPLAction_t*[EnvOMPLCfg.NumThetaDirs];
    EnvOMPLCfg.PredActionsV = new vector<EnvOMPLAction_t*> [EnvOMPLCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;

    //iterate over source angles
    for (int tind = 0; tind < EnvOMPLCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvOMPLCfg.NumThetaDirs);
        EnvOMPLCfg.ActionsV[tind] = new EnvOMPLAction_t[motionprimitiveV->size()];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvOMPLCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvOMPLCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvOMPLCfg.NumThetaDirs);

        //iterate over motion primitives
        for (size_t aind = 0; aind < motionprimitiveV->size(); aind++) {
            EnvOMPLCfg.ActionsV[tind][aind].aind = aind;
            EnvOMPLCfg.ActionsV[tind][aind].starttheta = tind;
            double mp_endx_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].x;
            double mp_endy_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].y;
            double mp_endtheta_rad =
                    motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].theta;

            double endx = sourcepose.x + (mp_endx_m * cos(sourcepose.theta) - mp_endy_m * sin(sourcepose.theta));
            double endy = sourcepose.y + (mp_endx_m * sin(sourcepose.theta) + mp_endy_m * cos(sourcepose.theta));

            int endx_c = CONTXY2DISC(endx, EnvOMPLCfg.cellsize_m);
            int endy_c = CONTXY2DISC(endy, EnvOMPLCfg.cellsize_m);

            EnvOMPLCfg.ActionsV[tind][aind].endtheta = ContTheta2Disc(mp_endtheta_rad + sourcepose.theta,
                                                                               EnvOMPLCfg.NumThetaDirs);
            EnvOMPLCfg.ActionsV[tind][aind].dX = endx_c;
            EnvOMPLCfg.ActionsV[tind][aind].dY = endy_c;
            if (EnvOMPLCfg.ActionsV[tind][aind].dY != 0 || EnvOMPLCfg.ActionsV[tind][aind].dX != 0)
                EnvOMPLCfg.ActionsV[tind][aind].cost = (int)(ceil(OMPL_COSTMULT_MTOMM
                    * EnvOMPLCfg.cellsize_m / EnvOMPLCfg.nominalvel_mpersecs
                    * sqrt((double)(EnvOMPLCfg.ActionsV[tind][aind].dX
                        * EnvOMPLCfg.ActionsV[tind][aind].dX + EnvOMPLCfg.ActionsV[tind][aind].dY
                        * EnvOMPLCfg.ActionsV[tind][aind].dY))));
            else
                //cost of turn in place
                EnvOMPLCfg.ActionsV[tind][aind].cost = (int)(OMPL_COSTMULT_MTOMM
                    * EnvOMPLCfg.timetoturn45degsinplace_secs
                    * fabs(computeMinUnsignedAngleDiff(mp_endtheta_rad, 0)) / (PI_CONST / 4.0));

            //compute and store interm points as well as intersecting cells
            EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvOMPLCfg.ActionsV[tind][aind].intermptV.clear();
            EnvOMPLCfg.ActionsV[tind][aind].interm3DcellsV.clear();
            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.theta = previnterm3Dcell.x = previnterm3Dcell.y = 0;

            for (int pind = 0; pind < (int)motionprimitiveV->at(aind).intermptV.size(); pind++) {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(aind).intermptV[pind];

                //rotate it appropriately
                double rotx = intermpt.x * cos(sourcepose.theta) - intermpt.y * sin(sourcepose.theta);
                double roty = intermpt.x * sin(sourcepose.theta) + intermpt.y * cos(sourcepose.theta);
                intermpt.x = rotx;
                intermpt.y = roty;
                intermpt.theta = normalizeAngle(sourcepose.theta + intermpt.theta);

                //store it (they are with reference to 0,0,stattheta (not
                //sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
                EnvOMPLCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);
            }
            //now compute the intersecting cells for this motion (including ignoring the source footprint)
            get_2d_motion_cells(EnvOMPLCfg.FootprintPolygon,
                                EnvOMPLCfg.ActionsV[tind][aind].intermptV,
                                &EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV,
                                EnvOMPLCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%d aind=%d: dX=%d dY=%d endtheta=%d (%.2f degs -> %.2f degs) "
                         "cost=%d (mprim: %.2f %.2f %.2f)\n",
                         tind,
                         (int)aind,
                         EnvOMPLCfg.ActionsV[tind][aind].dX,
                         EnvOMPLCfg.ActionsV[tind][aind].dY,
                         EnvOMPLCfg.ActionsV[tind][aind].endtheta,
                         sourcepose.theta * 180.0 / PI_CONST,
                         EnvOMPLCfg.ActionsV[tind][aind].intermptV[EnvOMPLCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180.0 / PI_CONST,
                         EnvOMPLCfg.ActionsV[tind][aind].cost,
                         mp_endx_m,
                         mp_endy_m,
                         mp_endtheta_rad);
#endif

            //add to the list of backward actions
            int targettheta = EnvOMPLCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvOMPLCfg.NumThetaDirs;
            EnvOMPLCfg.PredActionsV[targettheta].push_back(&(EnvOMPLCfg.ActionsV[tind][aind]));
        }
    }

    //set number of actions
    EnvOMPLCfg.actionwidth = motionprimitiveV->size();

    //now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}

//here motionprimitivevector contains actions for all angles
void EnvironmentOMPLTICE::PrecomputeActionswithCompleteMotionPrimitive(
        vector<SBPL_ompl_mprimitive>* motionprimitiveV)
{
    SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
    EnvOMPLCfg.ActionsV = new EnvOMPLAction_t*[EnvOMPLCfg.NumThetaDirs];
    EnvOMPLCfg.PredActionsV = new vector<EnvOMPLAction_t*> [EnvOMPLCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;

    if (motionprimitiveV->size() % EnvOMPLCfg.NumThetaDirs != 0) {
        SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
        throw new SBPL_Exception();
    }

    EnvOMPLCfg.actionwidth = ((int)motionprimitiveV->size()) / EnvOMPLCfg.NumThetaDirs;

    //iterate over source angles
    int maxnumofactions = 0;
    for (int tind = 0; tind < EnvOMPLCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvOMPLCfg.NumThetaDirs);

        EnvOMPLCfg.ActionsV[tind] = new EnvOMPLAction_t[EnvOMPLCfg.actionwidth];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvOMPLCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvOMPLCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvOMPLCfg.NumThetaDirs);

        //iterate over motion primitives
        int numofactions = 0;
        int aind = -1;
        for (int mind = 0; mind < (int)motionprimitiveV->size(); mind++) {
            //find a motion primitive for this angle
            if (motionprimitiveV->at(mind).starttheta_c != tind) continue;

            aind++;
            numofactions++;

            //action index
            EnvOMPLCfg.ActionsV[tind][aind].aind = aind;

            //start angle
            EnvOMPLCfg.ActionsV[tind][aind].starttheta = tind;

            //compute dislocation
            EnvOMPLCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
            EnvOMPLCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
            EnvOMPLCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;

            //compute and store interm points as well as intersecting cells
            EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvOMPLCfg.ActionsV[tind][aind].intermptV.clear();
            EnvOMPLCfg.ActionsV[tind][aind].interm3DcellsV.clear();

            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.x = 0;
            previnterm3Dcell.y = 0;

            // Compute all the intersected cells for this action (intermptV and interm3DcellsV)
            for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++) {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
                EnvOMPLCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

                // also compute the intermediate discrete cells if not there already
                sbpl_xy_theta_pt_t pose;
                pose.x = intermpt.x + sourcepose.x;
                pose.y = intermpt.y + sourcepose.y;
                pose.theta = intermpt.theta;

                sbpl_xy_theta_cell_t intermediate2dCell;
                intermediate2dCell.x = CONTXY2DISC(pose.x, EnvOMPLCfg.cellsize_m);
                intermediate2dCell.y = CONTXY2DISC(pose.y, EnvOMPLCfg.cellsize_m);
                intermediate2dCell.theta = ContTheta2Disc(pose.theta, EnvOMPLCfg.NumThetaDirs);

                // add unique cells to the list
                if (EnvOMPLCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 || intermediate2dCell.x
                    != previnterm3Dcell.x || intermediate2dCell.y != previnterm3Dcell.y) {
                    EnvOMPLCfg.ActionsV[tind][aind].interm3DcellsV.push_back(intermediate2dCell);
                }

                previnterm3Dcell = intermediate2dCell;
            }

            //compute linear and angular time
            double linear_distance = 0;
            for (unsigned int i = 1; i < EnvOMPLCfg.ActionsV[tind][aind].intermptV.size(); i++) {
                double x0 = EnvOMPLCfg.ActionsV[tind][aind].intermptV[i - 1].x;
                double y0 = EnvOMPLCfg.ActionsV[tind][aind].intermptV[i - 1].y;
                double x1 = EnvOMPLCfg.ActionsV[tind][aind].intermptV[i].x;
                double y1 = EnvOMPLCfg.ActionsV[tind][aind].intermptV[i].y;
                double dx = x1 - x0;
                double dy = y1 - y0;
                linear_distance += sqrt(dx * dx + dy * dy);
            }
            double linear_time = linear_distance / EnvOMPLCfg.nominalvel_mpersecs;
            double angular_distance =
                    fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(EnvOMPLCfg.ActionsV[tind][aind].endtheta,
                                                                    EnvOMPLCfg.NumThetaDirs),
                                                     DiscTheta2Cont(EnvOMPLCfg.ActionsV[tind][aind].starttheta,
                                                                    EnvOMPLCfg.NumThetaDirs)));
            double angular_time = angular_distance / ((PI_CONST / 4.0) /
                                  EnvOMPLCfg.timetoturn45degsinplace_secs);
            //make the cost the max of the two times
            EnvOMPLCfg.ActionsV[tind][aind].cost =
                    (int)(ceil(OMPL_COSTMULT_MTOMM * max(linear_time, angular_time)));
            //use any additional cost multiplier
            EnvOMPLCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

            //now compute the intersecting cells for this motion (including ignoring the source footprint)
            get_2d_motion_cells(EnvOMPLCfg.FootprintPolygon, motionprimitiveV->at(mind).intermptV,
                                &EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV,
                                EnvOMPLCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%2d aind=%2d: dX=%3d dY=%3d endtheta=%3d (%6.2f degs -> %6.2f degs) "
                         "cost=%4d (mprimID %3d: %3d %3d %3d) numofintermcells = %d numofintercells=%d\n",
                         tind,
                         aind,
                         EnvOMPLCfg.ActionsV[tind][aind].dX,
                         EnvOMPLCfg.ActionsV[tind][aind].dY,
                         EnvOMPLCfg.ActionsV[tind][aind].endtheta,
                         EnvOMPLCfg.ActionsV[tind][aind].intermptV[0].theta * 180 / PI_CONST,
                         EnvOMPLCfg.ActionsV[tind][aind].intermptV[EnvOMPLCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180 / PI_CONST, EnvOMPLCfg.ActionsV[tind][aind].cost,
                         motionprimitiveV->at(mind).motprimID, motionprimitiveV->at(mind).endcell.x,
                         motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.theta,
                         (int)EnvOMPLCfg.ActionsV[tind][aind].interm3DcellsV.size(),
                         (int)EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV.size());
#endif

            //add to the list of backward actions
            int targettheta = EnvOMPLCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvOMPLCfg.NumThetaDirs;
            EnvOMPLCfg.PredActionsV[targettheta].push_back(&(EnvOMPLCfg.ActionsV[tind][aind]));
        }

        if (maxnumofactions < numofactions) maxnumofactions = numofactions;
    }

    //at this point we don't allow nonuniform number of actions
    if (motionprimitiveV->size() != (size_t)(EnvOMPLCfg.NumThetaDirs * maxnumofactions)) {
        SBPL_ERROR("ERROR: nonuniform number of actions is not supported "
                   "(maxnumofactions=%d while motprims=%d thetas=%d\n",
                   maxnumofactions, (unsigned int)motionprimitiveV->size(), EnvOMPLCfg.NumThetaDirs);
        throw new SBPL_Exception();
    }

    //now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}

void EnvironmentOMPLTICE::DeprecatedPrecomputeActions()
{
    SBPL_PRINTF("Use of DeprecatedPrecomputeActions() is deprecated and probably doesn't work!\n");

    //construct list of actions
    SBPL_PRINTF("Pre-computing action data using the motion primitives for a 3D kinematic planning...\n");
    EnvOMPLCfg.ActionsV = new EnvOMPLAction_t*[EnvOMPLCfg.NumThetaDirs];
    EnvOMPLCfg.PredActionsV = new vector<EnvOMPLAction_t*> [EnvOMPLCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;
    //iterate over source angles
    for (int tind = 0; tind < EnvOMPLCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("processing angle %d\n", tind);
        EnvOMPLCfg.ActionsV[tind] = new EnvOMPLAction_t[EnvOMPLCfg.actionwidth];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvOMPLCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvOMPLCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvOMPLCfg.NumThetaDirs);

        //the construction assumes that the robot first turns and then goes along this new theta
        int aind = 0;
        for (; aind < 3; aind++) {
            EnvOMPLCfg.ActionsV[tind][aind].aind = aind;
            EnvOMPLCfg.ActionsV[tind][aind].starttheta = tind;
            //-1,0,1
            EnvOMPLCfg.ActionsV[tind][aind].endtheta = (tind + aind - 1) % EnvOMPLCfg.NumThetaDirs; 
            double angle = DiscTheta2Cont(EnvOMPLCfg.ActionsV[tind][aind].endtheta,
                                          EnvOMPLCfg.NumThetaDirs);
            EnvOMPLCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5 * (cos(angle) > 0 ? 1 : -1));
            EnvOMPLCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5 * (sin(angle) > 0 ? 1 : -1));
            EnvOMPLCfg.ActionsV[tind][aind].cost = (int)(ceil(OMPL_COSTMULT_MTOMM
                * EnvOMPLCfg.cellsize_m / EnvOMPLCfg.nominalvel_mpersecs
                * sqrt((double)(EnvOMPLCfg.ActionsV[tind][aind].dX
                    * EnvOMPLCfg.ActionsV[tind][aind].dX + EnvOMPLCfg.ActionsV[tind][aind].dY
                    * EnvOMPLCfg.ActionsV[tind][aind].dY))));

            //compute intersecting cells
            sbpl_xy_theta_pt_t pose;
            pose.x = DISCXY2CONT(EnvOMPLCfg.ActionsV[tind][aind].dX, EnvOMPLCfg.cellsize_m);
            pose.y = DISCXY2CONT(EnvOMPLCfg.ActionsV[tind][aind].dY, EnvOMPLCfg.cellsize_m);
            pose.theta = angle;
            EnvOMPLCfg.ActionsV[tind][aind].intermptV.clear();
            EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            get_2d_footprint_cells(EnvOMPLCfg.FootprintPolygon,
                                   &EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                                   EnvOMPLCfg.cellsize_m);
            RemoveSourceFootprint(sourcepose, &EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
            SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                        tind, aind, EnvOMPLCfg.ActionsV[tind][aind].endtheta, angle,
                        EnvOMPLCfg.ActionsV[tind][aind].dX, EnvOMPLCfg.ActionsV[tind][aind].dY,
                        EnvOMPLCfg.ActionsV[tind][aind].cost);
#endif

            //add to the list of backward actions
            int targettheta = EnvOMPLCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvOMPLCfg.NumThetaDirs;
            EnvOMPLCfg.PredActionsV[targettheta].push_back(&(EnvOMPLCfg.ActionsV[tind][aind]));
        }

        //decrease and increase angle without movement
        aind = 3;
        EnvOMPLCfg.ActionsV[tind][aind].aind = aind;
        EnvOMPLCfg.ActionsV[tind][aind].starttheta = tind;
        EnvOMPLCfg.ActionsV[tind][aind].endtheta = tind - 1;
        if (EnvOMPLCfg.ActionsV[tind][aind].endtheta < 0)
            EnvOMPLCfg.ActionsV[tind][aind].endtheta += EnvOMPLCfg.NumThetaDirs;
        EnvOMPLCfg.ActionsV[tind][aind].dX = 0;
        EnvOMPLCfg.ActionsV[tind][aind].dY = 0;
        EnvOMPLCfg.ActionsV[tind][aind].cost = (int)(OMPL_COSTMULT_MTOMM *
                                                              EnvOMPLCfg.timetoturn45degsinplace_secs);

        //compute intersecting cells
        sbpl_xy_theta_pt_t pose;
        pose.x = DISCXY2CONT(EnvOMPLCfg.ActionsV[tind][aind].dX, EnvOMPLCfg.cellsize_m);
        pose.y = DISCXY2CONT(EnvOMPLCfg.ActionsV[tind][aind].dY, EnvOMPLCfg.cellsize_m);
        pose.theta =
                DiscTheta2Cont(EnvOMPLCfg.ActionsV[tind][aind].endtheta, EnvOMPLCfg.NumThetaDirs);
        EnvOMPLCfg.ActionsV[tind][aind].intermptV.clear();
        EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV.clear();
        get_2d_footprint_cells(EnvOMPLCfg.FootprintPolygon,
                               &EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                               EnvOMPLCfg.cellsize_m);
        RemoveSourceFootprint(sourcepose, &EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
        SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                    tind, aind, EnvOMPLCfg.ActionsV[tind][aind].endtheta,
                    DiscTheta2Cont(EnvOMPLCfg.ActionsV[tind][aind].endtheta, EnvOMPLCfg.NumThetaDirs),
                    EnvOMPLCfg.ActionsV[tind][aind].dX, EnvOMPLCfg.ActionsV[tind][aind].dY,
                    EnvOMPLCfg.ActionsV[tind][aind].cost);
#endif

        //add to the list of backward actions
        int targettheta = EnvOMPLCfg.ActionsV[tind][aind].endtheta;
        if (targettheta < 0) targettheta = targettheta + EnvOMPLCfg.NumThetaDirs;
        EnvOMPLCfg.PredActionsV[targettheta].push_back(&(EnvOMPLCfg.ActionsV[tind][aind]));

        aind = 4;
        EnvOMPLCfg.ActionsV[tind][aind].aind = aind;
        EnvOMPLCfg.ActionsV[tind][aind].starttheta = tind;
        EnvOMPLCfg.ActionsV[tind][aind].endtheta = (tind + 1) % EnvOMPLCfg.NumThetaDirs;
        EnvOMPLCfg.ActionsV[tind][aind].dX = 0;
        EnvOMPLCfg.ActionsV[tind][aind].dY = 0;
        EnvOMPLCfg.ActionsV[tind][aind].cost = (int)(OMPL_COSTMULT_MTOMM *
                                                              EnvOMPLCfg.timetoturn45degsinplace_secs);

        //compute intersecting cells
        pose.x = DISCXY2CONT(EnvOMPLCfg.ActionsV[tind][aind].dX, EnvOMPLCfg.cellsize_m);
        pose.y = DISCXY2CONT(EnvOMPLCfg.ActionsV[tind][aind].dY, EnvOMPLCfg.cellsize_m);
        pose.theta =
                DiscTheta2Cont(EnvOMPLCfg.ActionsV[tind][aind].endtheta, EnvOMPLCfg.NumThetaDirs);
        EnvOMPLCfg.ActionsV[tind][aind].intermptV.clear();
        EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV.clear();
        get_2d_footprint_cells(EnvOMPLCfg.FootprintPolygon,
                               &EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                               EnvOMPLCfg.cellsize_m);
        RemoveSourceFootprint(sourcepose, &EnvOMPLCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
        SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                    tind, aind, EnvOMPLCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvOMPLCfg.ActionsV[tind][aind].endtheta, EnvOMPLCfg.NumThetaDirs),
                    EnvOMPLCfg.ActionsV[tind][aind].dX, EnvOMPLCfg.ActionsV[tind][aind].dY,
                    EnvOMPLCfg.ActionsV[tind][aind].cost);
#endif

        //add to the list of backward actions
        targettheta = EnvOMPLCfg.ActionsV[tind][aind].endtheta;
        if (targettheta < 0) targettheta = targettheta + EnvOMPLCfg.NumThetaDirs;
        EnvOMPLCfg.PredActionsV[targettheta].push_back(&(EnvOMPLCfg.ActionsV[tind][aind]));
    }

    //now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data\n");
}

void EnvironmentOMPLTICE::InitializeEnvConfig(vector<SBPL_ompl_mprimitive>* motionprimitiveV)
{
    //aditional to configuration file initialization of EnvOMPLCfg if necessary

    //dXY dirs
    EnvOMPLCfg.dXY[0][0] = -1;
    EnvOMPLCfg.dXY[0][1] = -1;
    EnvOMPLCfg.dXY[1][0] = -1;
    EnvOMPLCfg.dXY[1][1] = 0;
    EnvOMPLCfg.dXY[2][0] = -1;
    EnvOMPLCfg.dXY[2][1] = 1;
    EnvOMPLCfg.dXY[3][0] = 0;
    EnvOMPLCfg.dXY[3][1] = -1;
    EnvOMPLCfg.dXY[4][0] = 0;
    EnvOMPLCfg.dXY[4][1] = 1;
    EnvOMPLCfg.dXY[5][0] = 1;
    EnvOMPLCfg.dXY[5][1] = -1;
    EnvOMPLCfg.dXY[6][0] = 1;
    EnvOMPLCfg.dXY[6][1] = 0;
    EnvOMPLCfg.dXY[7][0] = 1;
    EnvOMPLCfg.dXY[7][1] = 1;

    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
    temppose.theta = 0.0;
    vector<sbpl_2Dcell_t> footprint;
    get_2d_footprint_cells(EnvOMPLCfg.FootprintPolygon, &footprint, temppose, EnvOMPLCfg.cellsize_m);
    SBPL_PRINTF("number of cells in footprint of the robot = %d\n", (unsigned int)footprint.size());

    for (vector<sbpl_2Dcell_t>::iterator it = footprint.begin(); it != footprint.end(); ++it) {
        SBPL_PRINTF("Footprint cell at (%d, %d)\n", it->x, it->y);
    }

#if DEBUG
    SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", (int)footprint.size());
    for(int i = 0; i < (int) footprint.size(); i++)
    {
        SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x, footprint.at(i).y,
                     DISCXY2CONT(footprint.at(i).x, EnvOMPLCfg.cellsize_m),
                     DISCXY2CONT(footprint.at(i).y, EnvOMPLCfg.cellsize_m));
    }
#endif

    if (motionprimitiveV == NULL)
        DeprecatedPrecomputeActions();
    else
        PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);
}

bool EnvironmentOMPLTICE::IsValidCell(int X, int Y)
{
    return (X >= 0 && X < EnvOMPLCfg.EnvWidth_c && Y >= 0 && Y < EnvOMPLCfg.EnvHeight_c &&
            EnvOMPLCfg.Grid2D[X][Y] < EnvOMPLCfg.obsthresh);
}

bool EnvironmentOMPLTICE::IsWithinMapCell(int X, int Y)
{
    return (X >= 0 && X < EnvOMPLCfg.EnvWidth_c && Y >= 0 && Y < EnvOMPLCfg.EnvHeight_c);
}

bool EnvironmentOMPLTICE::IsValidConfiguration(int X, int Y, int Theta)
{
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    pose.x = DISCXY2CONT(X, EnvOMPLCfg.cellsize_m);
    pose.y = DISCXY2CONT(Y, EnvOMPLCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(Theta, EnvOMPLCfg.NumThetaDirs);

    //compute footprint cells
    get_2d_footprint_cells(EnvOMPLCfg.FootprintPolygon, &footprint, pose, EnvOMPLCfg.cellsize_m);

    //iterate over all footprint cells
    for (int find = 0; find < (int)footprint.size(); find++) {
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;

        if (x < 0 || x >= EnvOMPLCfg.EnvWidth_c || y < 0 || y >= EnvOMPLCfg.EnvHeight_c ||
            EnvOMPLCfg.Grid2D[x][y] >= EnvOMPLCfg.obsthresh)
        {
            return false;
        }
    }

    return true;
}

double EnvironmentOMPLTICE::ShAngDist(double from, double to)
{
    double angle = to - from;

    double a = fmod(fmod(angle, 2*M_PI) + 2*M_PI, 2*M_PI);

    if(a > M_PI)
        a -= 2*M_PI;

    return a;
}

int EnvironmentOMPLTICE::GetActionCost(int SourceX, int SourceY, int SourceTheta,
                                                EnvOMPLAction_t* action)
{
    sbpl_2Dcell_t cell;
    sbpl_xy_theta_cell_t interm3Dcell;
    int i;
    unsigned int ompl_action_cost = 0;
    double ompl_parent_x,ompl_parent_y,ompl_parent_theta;
    double ompl_child_x,ompl_child_y,ompl_child_theta;

    // printf("Ok here is the source state entering the GetAction\n");
    // printf("Source.x = %f\n",DISCXY2CONT(SourceX, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_x);
    // printf("Source.y = %f\n",DISCXY2CONT(SourceY, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_y);
    // printf("Source.theta = %f\n\n",DiscTheta2Cont(NORMALIZEDISCTHETA(SourceTheta, EnvOMPLCfg.NumThetaDirs), EnvOMPLCfg.NumThetaDirs));

    //TODO - go over bounding box (minpt and maxpt) to test validity and skip
    //testing boundaries below, also order intersect cells so that the four
    //farthest pts go first

    if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
    if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;

    if (EnvOMPLCfg.Grid2D[SourceX + action->dX][SourceY + action->dY] >=
        EnvOMPLCfg.cost_inscribed_thresh) 
    {
        return INFINITECOST;
    }

    ompl::base::State *parent_state = si_->getStateSpace()->allocState();
    ompl::base::State *child_state = si_->getStateSpace()->allocState();

    double source_th = DiscTheta2Cont(SourceTheta, EnvOMPLCfg.NumThetaDirs);
    double end_th = DiscTheta2Cont(action->endtheta, EnvOMPLCfg.NumThetaDirs);

    //double del_th = (ShAngDist(source_th,end_th))/action->interm3DcellsV.size();
    double del_th = (angles::shortest_angular_distance(source_th, end_th))/action->interm3DcellsV.size();

    ompl_parent_x = DISCXY2CONT(SourceX, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_x;
    ompl_parent_y = DISCXY2CONT(SourceY, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_y;
    ompl_parent_theta = source_th + (i+1)*del_th;

    parent_state->as<ompl::base::SE2StateSpace::StateType>()->setX(ompl_parent_x);
    parent_state->as<ompl::base::SE2StateSpace::StateType>()->setY(ompl_parent_y);
    parent_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(angles::normalize_angle(ompl_parent_theta));

    //need to iterate over discretized center cells and compute cost based on them
    unsigned char maxcellcost = 0;
    for (i = 0; i < (int)action->interm3DcellsV.size(); i++) {
        interm3Dcell = action->interm3DcellsV.at(i);
        interm3Dcell.x = interm3Dcell.x + SourceX;
        interm3Dcell.y = interm3Dcell.y + SourceY;

        if (interm3Dcell.x < 0 || interm3Dcell.x >= EnvOMPLCfg.EnvWidth_c || interm3Dcell.y < 0
            || interm3Dcell.y >= EnvOMPLCfg.EnvHeight_c) return INFINITECOST;

        maxcellcost = __max(maxcellcost, EnvOMPLCfg.Grid2D[interm3Dcell.x][interm3Dcell.y]);

        //check that the robot is NOT in the cell at which there is no valid orientation
        if (maxcellcost >= EnvOMPLCfg.cost_inscribed_thresh) return INFINITECOST;
    }

    //check collisions that for the particular footprint orientation along the action
    // if (EnvOMPLCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >=
    //     EnvOMPLCfg.cost_possibly_circumscribed_thresh)
    // {
    //     checks++;

    //     for (i = 0; i < (int)action->intersectingcellsV.size(); i++) {
    //         //get the cell in the map
    //         cell = action->intersectingcellsV.at(i);
    //         cell.x = cell.x + SourceX;
    //         cell.y = cell.y + SourceY;

    //         //check validity
    //         if (!IsValidCell(cell.x, cell.y)) return INFINITECOST;

    //         //if(EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] > currentmaxcost)
    //         ////cost computation changed: cost = max(cost of centers of the
    //         //robot along action)
    //         //  currentmaxcost = EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y];
    //         //  //intersecting cells are only used for collision checking
    //     }
    // }

    //OMPL setup to include collision check
    for (i = 0; i < (int)action->interm3DcellsV.size(); i++) {
        interm3Dcell = action->interm3DcellsV.at(i);
        interm3Dcell.x = interm3Dcell.x + SourceX;
        interm3Dcell.y = interm3Dcell.y + SourceY;

        ompl_child_x = DISCXY2CONT(interm3Dcell.x, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_x;
        ompl_child_y = DISCXY2CONT(interm3Dcell.y, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_y;
        ompl_child_theta = source_th + (i+1)*del_th;

        child_state->as<ompl::base::SE2StateSpace::StateType>()->setX(ompl_child_x);
        child_state->as<ompl::base::SE2StateSpace::StateType>()->setY(ompl_child_y);
        child_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(angles::normalize_angle(ompl_child_theta));

        ompl_action_cost += GetContEdgeCost(parent_state, child_state);

        //check validity
        if(!(si_->getStateValidityChecker())->isValid(child_state) ){
           return INFINITECOST;
        }
        
        ompl_parent_x = DISCXY2CONT(interm3Dcell.x, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_x;
        ompl_parent_y = DISCXY2CONT(interm3Dcell.y, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_y;
        ompl_parent_theta = source_th + (i+1)*del_th;

        parent_state->as<ompl::base::SE2StateSpace::StateType>()->setX(ompl_parent_x);
        parent_state->as<ompl::base::SE2StateSpace::StateType>()->setY(ompl_parent_y);
        parent_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(angles::normalize_angle(ompl_parent_theta));
    }
    // printf("Action cost is %d\n", ompl_action_cost);
    // printf("Parent state x is %f y is %f and theta is %f\n",parent_state->as<ompl::base::SE2StateSpace::StateType>()->getX(),
    //                                                       parent_state->as<ompl::base::SE2StateSpace::StateType>()->getY(),
    //                                                       parent_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());

    // printf("child state x is %f y is %f and theta is %f\n",child_state->as<ompl::base::SE2StateSpace::StateType>()->getX(),
    //                                                       child_state->as<ompl::base::SE2StateSpace::StateType>()->getY(),
    //                                                       child_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());


    // printf("The sbpl action cost is %d\n", action->cost);    

    // printf("The ompl action cost is %d\n", ompl_action_cost);

    si_->getStateSpace()->freeState(parent_state);
    si_->getStateSpace()->freeState(child_state);

    //to ensure consistency of h2D:
    maxcellcost = __max(maxcellcost, EnvOMPLCfg.Grid2D[SourceX][SourceY]);
    int currentmaxcost =
            (int)__max(maxcellcost, EnvOMPLCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

    //return action->cost * (currentmaxcost + 1); //use cell cost as multiplicative factor
    return ompl_action_cost * (currentmaxcost + 1); //use cell cost as multiplicative factor
}

double EnvironmentOMPLTICE::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2));
    return EnvOMPLCfg.cellsize_m * sqrt((double)sqdist);
}

//calculates a set of cells that correspond to the specified footprint
//adds points to it (does not clear it beforehand)
void EnvironmentOMPLTICE::CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, vector<sbpl_2Dcell_t>* footprint,
                                                             const vector<sbpl_2Dpt_t>& FootprintPolygon)
{
    int pind;

#if DEBUG
    //  SBPL_PRINTF("---Calculating Footprint for Pose: %f %f %f---\n",
    //   pose.x, pose.y, pose.theta);
#endif

    //handle special case where footprint is just a point
    if (FootprintPolygon.size() <= 1) {
        sbpl_2Dcell_t cell;
        cell.x = CONTXY2DISC(pose.x, EnvOMPLCfg.cellsize_m);
        cell.y = CONTXY2DISC(pose.y, EnvOMPLCfg.cellsize_m);

        for (pind = 0; pind < (int)footprint->size(); pind++) {
            if (cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y) break;
        }
        if (pind == (int)footprint->size()) footprint->push_back(cell);
        return;
    }

    vector<sbpl_2Dpt_t> bounding_polygon;
    unsigned int find;
    double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
    sbpl_2Dpt_t pt(0, 0);
    for (find = 0; find < FootprintPolygon.size(); find++) {
        //rotate and translate the corner of the robot
        pt = FootprintPolygon[find];

        //rotate and translate the point
        sbpl_2Dpt_t corner;
        corner.x = cos(pose.theta) * pt.x - sin(pose.theta) * pt.y + pose.x;
        corner.y = sin(pose.theta) * pt.x + cos(pose.theta) * pt.y + pose.y;
        bounding_polygon.push_back(corner);
#if DEBUG
        //    SBPL_PRINTF("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
        if (corner.x < min_x || find == 0) {
            min_x = corner.x;
        }
        if (corner.x > max_x || find == 0) {
            max_x = corner.x;
        }
        if (corner.y < min_y || find == 0) {
            min_y = corner.y;
        }
        if (corner.y > max_y || find == 0) {
            max_y = corner.y;
        }
    }

#if DEBUG
    //  SBPL_PRINTF("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
    //initialize previous values to something that will fail the if condition during the first iteration in the for loop
    int prev_discrete_x = CONTXY2DISC(pt.x, EnvOMPLCfg.cellsize_m) + 1;
    int prev_discrete_y = CONTXY2DISC(pt.y, EnvOMPLCfg.cellsize_m) + 1;
    int prev_inside = 0;
    int discrete_x;
    int discrete_y;

    for (double x = min_x; x <= max_x; x += EnvOMPLCfg.cellsize_m / 3) {
        for (double y = min_y; y <= max_y; y += EnvOMPLCfg.cellsize_m / 3) {
            pt.x = x;
            pt.y = y;
            discrete_x = CONTXY2DISC(pt.x, EnvOMPLCfg.cellsize_m);
            discrete_y = CONTXY2DISC(pt.y, EnvOMPLCfg.cellsize_m);

            //see if we just tested this point
            if (discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside == 0) {

#if DEBUG
                //      SBPL_PRINTF("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif

                if (IsInsideFootprint(pt, &bounding_polygon)) {
                    //convert to a grid point

#if DEBUG
                    //          SBPL_PRINTF("Pt Inside %f %f\n", pt.x, pt.y);
#endif

                    sbpl_2Dcell_t cell;
                    cell.x = discrete_x;
                    cell.y = discrete_y;

                    //insert point if not there already
                    int pind = 0;
                    for (pind = 0; pind < (int)footprint->size(); pind++) {
                        if (cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y) break;
                    }
                    if (pind == (int)footprint->size()) footprint->push_back(cell);

                    prev_inside = 1;

#if DEBUG
                    //          SBPL_PRINTF("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
                }
                else {
                    prev_inside = 0;
                }

            }
            else {
#if DEBUG
                //SBPL_PRINTF("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
            }

            prev_discrete_x = discrete_x;
            prev_discrete_y = discrete_y;
        }//over x_min...x_max
    }
}

//calculates a set of cells that correspond to the footprint of the base
//adds points to it (does not clear it beforehand) 
void EnvironmentOMPLTICE::CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, vector<sbpl_2Dcell_t>* footprint)
{
    CalculateFootprintForPose(pose, footprint, EnvOMPLCfg.FootprintPolygon);
}

//removes a set of cells that correspond to the specified footprint at the sourcepose
//adds points to it (does not clear it beforehand) 
void EnvironmentOMPLTICE::RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose,
                                                         vector<sbpl_2Dcell_t>* footprint,
                                                         const vector<sbpl_2Dpt_t>& FootprintPolygon)
{
    vector<sbpl_2Dcell_t> sourcefootprint;

    //compute source footprint
    get_2d_footprint_cells(FootprintPolygon, &sourcefootprint, sourcepose, EnvOMPLCfg.cellsize_m);

    //now remove the source cells from the footprint
    for (int sind = 0; sind < (int)sourcefootprint.size(); sind++) {
        for (int find = 0; find < (int)footprint->size(); find++) {
            if (sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y
                == footprint->at(find).y) {
                footprint->erase(footprint->begin() + find);
                break;
            }
        }//over footprint
    }//over source
}

//removes a set of cells that correspond to the footprint of the base at the sourcepose
//adds points to it (does not clear it beforehand) 
void EnvironmentOMPLTICE::RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose,
                                                         vector<sbpl_2Dcell_t>* footprint)
{
    RemoveSourceFootprint(sourcepose, footprint, EnvOMPLCfg.FootprintPolygon);
}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentOMPLTICE::EnsureHeuristicsUpdated(bool bGoalHeuristics)
{
    if (bNeedtoRecomputeStartHeuristics && !bGoalHeuristics) {
        grid2Dsearchfromstart->search(EnvOMPLCfg.Grid2D, EnvOMPLCfg.cost_inscribed_thresh,
                                      EnvOMPLCfg.StartX_c, EnvOMPLCfg.StartY_c,
                                      EnvOMPLCfg.EndX_c, EnvOMPLCfg.EndY_c,
                                      SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeStartHeuristics = false;
        SBPL_PRINTF("2dsolcost_infullunits=%d\n",
                    (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvOMPLCfg.EndX_c,
                                                                                   EnvOMPLCfg.EndY_c) /
                          EnvOMPLCfg.nominalvel_mpersecs));

    }

    if (bNeedtoRecomputeGoalHeuristics && bGoalHeuristics) {
        grid2Dsearchfromgoal->search(EnvOMPLCfg.Grid2D, EnvOMPLCfg.cost_inscribed_thresh,
                                     EnvOMPLCfg.EndX_c, EnvOMPLCfg.EndY_c,
                                     EnvOMPLCfg.StartX_c, EnvOMPLCfg.StartY_c,
                                     SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeGoalHeuristics = false;
        SBPL_PRINTF("2dsolcost_infullunits=%d\n",
                    (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvOMPLCfg.StartX_c,
                                                                                  EnvOMPLCfg.StartY_c) /
                          EnvOMPLCfg.nominalvel_mpersecs));
    }
}

void EnvironmentOMPLTICE::ComputeHeuristicValues()
{
    //whatever necessary pre-computation of heuristic values is done here
    SBPL_PRINTF("Precomputing heuristics...\n");

    //allocated 2D grid searches
    grid2Dsearchfromstart = new SBPL2DGridSearch(EnvOMPLCfg.EnvWidth_c, EnvOMPLCfg.EnvHeight_c,
                                                 (float)EnvOMPLCfg.cellsize_m);
    grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvOMPLCfg.EnvWidth_c, EnvOMPLCfg.EnvHeight_c,
                                                (float)EnvOMPLCfg.cellsize_m);

    //set OPEN type to sliding buckets
    grid2Dsearchfromstart->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);
    grid2Dsearchfromgoal->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);

    SBPL_PRINTF("done\n");
}

//------------debugging functions---------------------------------------------
bool EnvironmentOMPLTICE::CheckQuant(FILE* fOut)
{
    for (double theta = -10; theta < 10; theta += 2.0 * PI_CONST / EnvOMPLCfg.NumThetaDirs * 0.01) {
        int nTheta = ContTheta2Disc(theta, EnvOMPLCfg.NumThetaDirs);
        double newTheta = DiscTheta2Cont(nTheta, EnvOMPLCfg.NumThetaDirs);
        int nnewTheta = ContTheta2Disc(newTheta, EnvOMPLCfg.NumThetaDirs);

        SBPL_FPRINTF(fOut, "theta=%f(%f)->%d->%f->%d\n", theta, theta * 180 / PI_CONST, nTheta, newTheta, nnewTheta);

        if (nTheta != nnewTheta) {
            SBPL_ERROR("ERROR: invalid quantization\n");
            return false;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------

bool EnvironmentOMPLTICE::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV,
                                                 const char* sMotPrimFile)
{
    EnvOMPLCfg.FootprintPolygon = perimeterptsV;

    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw new SBPL_Exception();
    }
    ReadConfiguration(sEnvFile);
    fclose(fCfg);

    if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {
            SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
            throw new SBPL_Exception();
        }
        if (ReadMotionPrimitives(fMotPrim) == false) {
            SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
            throw new SBPL_Exception();
        }
        InitGeneral(&EnvOMPLCfg.mprimV);
        fclose(fMotPrim);
    }
    else
        InitGeneral( NULL);

    SBPL_PRINTF("size of env: %d by %d\n", EnvOMPLCfg.EnvWidth_c, EnvOMPLCfg.EnvHeight_c);

    return true;
}

bool EnvironmentOMPLTICE::InitializeEnv(const char* sEnvFile)
{
    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw new SBPL_Exception();
    }
    ReadConfiguration(sEnvFile);
    fclose(fCfg);

    InitGeneral( NULL);

    return true;
}

bool EnvironmentOMPLTICE::InitializeEnv(int width, int height, const vector<sbpl_2Dpt_t> & perimeterptsV,
                                                 double cellsize_m, double nominalvel_mpersecs,
                                                 double timetoturn45degsinplace_secs, unsigned char obsthresh,
                                                 const char* sMotPrimFile, EnvOMPL_InitParms params)
{
    EnvOMPLCfg.NumThetaDirs = params.numThetas;

    return InitializeEnv(width, height, params.mapdata, params.startx, params.starty, params.starttheta, params.goalx,
                         params.goaly, params.goaltheta, params.goaltol_x, params.goaltol_y, params.goaltol_theta,
                         perimeterptsV, cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh,
                         sMotPrimFile);
}

bool EnvironmentOMPLTICE::InitializeEnv(int width, int height, const unsigned char* mapdata, double startx,
                                                 double starty, double starttheta, double goalx, double goaly,
                                                 double goaltheta, double goaltol_x, double goaltol_y,
                                                 double goaltol_theta, const vector<sbpl_2Dpt_t> & perimeterptsV,
                                                 double cellsize_m, double nominalvel_mpersecs,
                                                 double timetoturn45degsinplace_secs, unsigned char obsthresh,
                                                 const char* sMotPrimFile)
{
    SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f "
                "goalx=%.3f %.3f %.3f cellsize=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
                width, height, startx, starty, starttheta, goalx, goaly, goaltheta, cellsize_m, nominalvel_mpersecs,
                timetoturn45degsinplace_secs, obsthresh);

    SBPL_PRINTF("NOTE: goaltol parameters currently unused\n");

    SBPL_PRINTF("perimeter has size=%d\n", (unsigned int)perimeterptsV.size());

    for (int i = 0; i < (int)perimeterptsV.size(); i++) {
        SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
    }

    EnvOMPLCfg.obsthresh = obsthresh;

    //TODO - need to set the tolerance as well

    SetConfiguration(width, height, mapdata, CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m),
                     ContTheta2Disc(starttheta, EnvOMPLCfg.NumThetaDirs), CONTXY2DISC(goalx, cellsize_m),
                     CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, EnvOMPLCfg.NumThetaDirs),
                     cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

    if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {

            SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
            throw new SBPL_Exception();
        }

        if (ReadMotionPrimitives(fMotPrim) == false) {
            SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
            throw new SBPL_Exception();
        }
        fclose(fMotPrim);
    }

    if (EnvOMPLCfg.mprimV.size() != 0) {
        InitGeneral(&EnvOMPLCfg.mprimV);
    }
    else
        InitGeneral( NULL);

    return true;
}

bool EnvironmentOMPLTICE::InitGeneral(vector<SBPL_ompl_mprimitive>* motionprimitiveV)
{
    //Initialize other parameters of the environment
    InitializeEnvConfig(motionprimitiveV);

    //initialize Environment
    InitializeEnvironment();

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

bool EnvironmentOMPLTICE::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = EnvOMPL.goalstateid;
    MDPCfg->startstateid = EnvOMPL.startstateid;

    return true;
}

void EnvironmentOMPLTICE::PrintHeuristicValues()
{
//#ifndef ROS
    const char* heur = "heur.txt";
//#endif
    FILE* fHeur = fopen(heur, "w");
    if (fHeur == NULL) {
        SBPL_ERROR("ERROR: could not open debug file to write heuristic\n");
        throw new SBPL_Exception();
    }
    SBPL2DGridSearch* grid2Dsearch = NULL;

    for (int i = 0; i < 2; i++) {
        if (i == 0 && grid2Dsearchfromstart != NULL) {
            grid2Dsearch = grid2Dsearchfromstart;
            //fprintf(fHeur, "start heuristics:\n");
            continue;
        }
        else if (i == 1 && grid2Dsearchfromgoal != NULL) {
            grid2Dsearch = grid2Dsearchfromgoal;
            //fprintf(fHeur, "goal heuristics:\n");
        }
        else
            continue;

        for (int y = 0; y < EnvOMPLCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvOMPLCfg.EnvWidth_c; x++) {
                if (grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y) < INFINITECOST)
                    fprintf(fHeur, "%5d ", grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y));
                else
                    fprintf(fHeur, "%5d ", 0);
            }
            fprintf(fHeur, "\n");
        }
    }
    fclose(fHeur);
}

void EnvironmentOMPLTICE::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvOMPL... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentOMPLTICE::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    GetSuccs(SourceStateID, SuccIDV, CostV, NULL);
}
void EnvironmentOMPLTICE::GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
    GetLazySuccs(SourceStateID, SuccIDV, CostV, isTrueCost, NULL);
}
void EnvironmentOMPLTICE::GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){
    GetSuccsWithUniqueIds(SourceStateID, SuccIDV, CostV, NULL);
}
void EnvironmentOMPLTICE::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
    GetLazySuccsWithUniqueIds(SourceStateID, SuccIDV, CostV, isTrueCost, NULL);
}

const EnvOMPLConfig_t* EnvironmentOMPLTICE::GetEnvNavConfig()
{
    return &EnvOMPLCfg;
}

bool EnvironmentOMPLTICE::UpdateCost(int x, int y, unsigned char newcost)
{
#if DEBUG
    //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvOMPLCfg.Grid2D[x][y], newcost);
#endif

    EnvOMPLCfg.Grid2D[x][y] = newcost;

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

bool EnvironmentOMPLTICE::SetMap(const unsigned char* mapdata)
{
    int xind = -1, yind = -1;

    for (xind = 0; xind < EnvOMPLCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvOMPLCfg.EnvHeight_c; yind++) {
            EnvOMPLCfg.Grid2D[xind][yind] = mapdata[xind + yind * EnvOMPLCfg.EnvWidth_c];
        }
    }

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

void EnvironmentOMPLTICE::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvOMPL. configuration

    SBPL_ERROR("ERROR in EnvOMPL... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentOMPLTICE::Set2DBlockSize(int BlockSize)
{
    blocksize = BlockSize;
}

void EnvironmentOMPLTICE::Set2DBucketSize(int BucketSize)
{
    bucketsize = BucketSize;
}

void EnvironmentOMPLTICE::PrintTimeStat(FILE* fOut)
{
#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, "
                 "time_getsuccs = %f\n",
                 time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
                 time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}

bool EnvironmentOMPLTICE::IsObstacle(int x, int y)
{
#if DEBUG
    SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvOMPLCfg.Grid2D[x][y]);
#endif

    return (EnvOMPLCfg.Grid2D[x][y] >= EnvOMPLCfg.obsthresh);
}

void EnvironmentOMPLTICE::GetEnvParms(int *size_x, int *size_y, int* num_thetas, double* startx,
                                               double* starty, double*starttheta, double* goalx, double* goaly,
                                               double* goaltheta, double* cellsize_m, double* nominalvel_mpersecs,
                                               double* timetoturn45degsinplace_secs, unsigned char* obsthresh, vector<
                                                   SBPL_ompl_mprimitive>* mprimitiveV)
{
    *num_thetas = EnvOMPLCfg.NumThetaDirs;
    GetEnvParms(size_x, size_y, startx, starty, starttheta, goalx, goaly, goaltheta, cellsize_m, nominalvel_mpersecs,
                timetoturn45degsinplace_secs, obsthresh, mprimitiveV);
}

void EnvironmentOMPLTICE::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty,
                                               double*starttheta, double* goalx, double* goaly, double* goaltheta,
                                               double* cellsize_m, double* nominalvel_mpersecs,
                                               double* timetoturn45degsinplace_secs, unsigned char* obsthresh,
                                               vector<SBPL_ompl_mprimitive>* mprimitiveV)
{
    *size_x = EnvOMPLCfg.EnvWidth_c;
    *size_y = EnvOMPLCfg.EnvHeight_c;

    *startx = DISCXY2CONT(EnvOMPLCfg.StartX_c, EnvOMPLCfg.cellsize_m);
    *starty = DISCXY2CONT(EnvOMPLCfg.StartY_c, EnvOMPLCfg.cellsize_m);
    *starttheta = DiscTheta2Cont(EnvOMPLCfg.StartTheta, EnvOMPLCfg.NumThetaDirs);
    *goalx = DISCXY2CONT(EnvOMPLCfg.EndX_c, EnvOMPLCfg.cellsize_m);
    *goaly = DISCXY2CONT(EnvOMPLCfg.EndY_c, EnvOMPLCfg.cellsize_m);
    *goaltheta = DiscTheta2Cont(EnvOMPLCfg.EndTheta, EnvOMPLCfg.NumThetaDirs);

    *cellsize_m = EnvOMPLCfg.cellsize_m;
    *nominalvel_mpersecs = EnvOMPLCfg.nominalvel_mpersecs;
    *timetoturn45degsinplace_secs = EnvOMPLCfg.timetoturn45degsinplace_secs;

    *obsthresh = EnvOMPLCfg.obsthresh;

    *mprimitiveV = EnvOMPLCfg.mprimV;
}

bool EnvironmentOMPLTICE::PoseContToDisc(double px, double py, double pth, int &ix, int &iy, int &ith) const
{
    ix = CONTXY2DISC(px, EnvOMPLCfg.cellsize_m);
    iy = CONTXY2DISC(py, EnvOMPLCfg.cellsize_m);
    ith = ContTheta2Disc(pth, EnvOMPLCfg.NumThetaDirs); // ContTheta2Disc() normalizes the angle
    return (pth >= -2 * PI_CONST) && (pth <= 2 * PI_CONST) && (ix >= 0) && (ix < EnvOMPLCfg.EnvWidth_c) &&
           (iy >= 0) && (iy < EnvOMPLCfg.EnvHeight_c);
}

bool EnvironmentOMPLTICE::PoseDiscToCont(int ix, int iy, int ith, double &px, double &py, double &pth) const
{
    px = DISCXY2CONT(ix, EnvOMPLCfg.cellsize_m);
    py = DISCXY2CONT(iy, EnvOMPLCfg.cellsize_m);
    pth = normalizeAngle(DiscTheta2Cont(ith, EnvOMPLCfg.NumThetaDirs));
    return (ith >= 0) && (ith < EnvOMPLCfg.NumThetaDirs) && (ix >= 0) &&
           (ix < EnvOMPLCfg.EnvWidth_c) && (iy >= 0) && (iy < EnvOMPLCfg.EnvHeight_c);
}

unsigned char EnvironmentOMPLTICE::GetMapCost(int x, int y)
{
    return EnvOMPLCfg.Grid2D[x][y];
}

bool EnvironmentOMPLTICE::SetEnvParameter(const char* parameter, int value)
{
    if (EnvOMPL.bInitialized == true) {
        SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
        return false;
    }

    SBPL_PRINTF("setting parameter %s to %d\n", parameter, value);

    if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvOMPLCfg.cost_inscribed_thresh = (unsigned char)value;
    }
    else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvOMPLCfg.cost_possibly_circumscribed_thresh = value;
    }
    else if (strcmp(parameter, "cost_obsthresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvOMPLCfg.obsthresh = (unsigned char)value;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        return false;
    }

    return true;
}

int EnvironmentOMPLTICE::GetEnvParameter(const char* parameter)
{
    if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
        return (int)EnvOMPLCfg.cost_inscribed_thresh;
    }
    else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
        return (int)EnvOMPLCfg.cost_possibly_circumscribed_thresh;
    }
    else if (strcmp(parameter, "cost_obsthresh") == 0) {
        return (int)EnvOMPLCfg.obsthresh;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        throw new SBPL_Exception();
    }
}

//------------------------------------------------------------------------------

//-----------------XYTHETA Enivornment (child) class---------------------------

EnvironmentOMPL::~EnvironmentOMPL()
{
    SBPL_PRINTF("destroying OMPL\n");

    //delete the states themselves first
    for (int i = 0; i < (int)StateID2CoordTable.size(); i++) {
        delete StateID2CoordTable.at(i);
        StateID2CoordTable.at(i) = NULL;
    }
    StateID2CoordTable.clear();

    //delete hashtable
    if (Coord2StateIDHashTable != NULL) {
        delete[] Coord2StateIDHashTable;
        Coord2StateIDHashTable = NULL;
    }
    if (Coord2StateIDHashTable_lookup != NULL) {
        delete[] Coord2StateIDHashTable_lookup;
        Coord2StateIDHashTable_lookup = NULL;
    }
}

void EnvironmentOMPL::GetCoordFromState(int stateID, int& x, int& y, int& theta) const
{
    EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    x = HashEntry->X;
    y = HashEntry->Y;
    theta = HashEntry->Theta;
}

int EnvironmentOMPL::GetStateFromCoord(int x, int y, int theta)
{
    EnvOMPLHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    return OutHashEntry->stateID;
}

void EnvironmentOMPL::GetActionsFromStateIDPath(vector<int>* stateIDPath, 
                                                             vector<EnvOMPLAction_t>* action_list)
{
    vector<EnvOMPLAction_t*> actionV;
    vector<int> CostV;
    vector<int> SuccIDV;
    int targetx_c, targety_c, targettheta_c;
    int sourcex_c, sourcey_c, sourcetheta_c;

    SBPL_PRINTF("checks=%ld\n", checks);

    action_list->clear();

    for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

        //get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

        for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1) {
            SBPL_ERROR("ERROR: successor not found for transition:\n");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
            SBPL_PRINTF("%d %d %d -> %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c, targetx_c, targety_c,
                        targettheta_c);
            throw new SBPL_Exception();
        }
        
#if DEBUG
        SBPL_FPRINTF(fDeb, "Start: %.3f %.3f %.3f Target: %.3f %.3f %.3f Prim ID, Start Theta: %d %d\n",
            sourcex_c, sourcey_c, sourcetheta_c,
            targetx_c, targety_c, targettheta_c,
            actionV[bestsind]->aind, actionV[bestsind]->starttheta);
#endif
        
        action_list->push_back(*(actionV[bestsind]));
    }
}

void EnvironmentOMPL::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath,
                                                                 vector<sbpl_xy_theta_pt_t>* xythetaPath)
{
    vector<EnvOMPLAction_t*> actionV;
    vector<int> CostV;
    vector<int> SuccIDV;
    int targetx_c, targety_c, targettheta_c;
    int sourcex_c, sourcey_c, sourcetheta_c;

    SBPL_PRINTF("checks=%ld\n", checks);

    xythetaPath->clear();

#if DEBUG
    SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

    for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
#endif

        //get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
        GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
        SBPL_FPRINTF(fDeb, "looking for %d %d %d -> %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c,
                     targetx_c, targety_c, targettheta_c, (int)SuccIDV.size());
#endif

        for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
#if DEBUG
            int x_c, y_c, theta_c;
            GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c);
            SBPL_FPRINTF(fDeb, "succ: %d %d %d\n", x_c, y_c, theta_c);
#endif

            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1) {
            SBPL_ERROR("ERROR: successor not found for transition:\n");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
            SBPL_PRINTF("%d %d %d -> %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c, targetx_c, targety_c,
                        targettheta_c);
            throw new SBPL_Exception();
        }

        //now push in the actual path
        int sourcex_c, sourcey_c, sourcetheta_c;
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
        double sourcex, sourcey;
        sourcex = DISCXY2CONT(sourcex_c, EnvOMPLCfg.cellsize_m);
        sourcey = DISCXY2CONT(sourcey_c, EnvOMPLCfg.cellsize_m);
        //TODO - when there are no motion primitives we should still print source state
        for (int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size()) - 1; ipind++) {
            //translate appropriately
            sbpl_xy_theta_pt_t intermpt = actionV[bestsind]->intermptV[ipind];
            intermpt.x += sourcex;
            intermpt.y += sourcey;

#if DEBUG
            int nx = CONTXY2DISC(intermpt.x, EnvOMPLCfg.cellsize_m);
            int ny = CONTXY2DISC(intermpt.y, EnvOMPLCfg.cellsize_m);
            SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ",
                intermpt.x, intermpt.y, intermpt.theta,
                nx, ny,
                ContTheta2Disc(intermpt.theta, EnvOMPLCfg.NumThetaDirs), EnvOMPLCfg.Grid2D[nx][ny]);
            if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
            else SBPL_FPRINTF(fDeb, "\n");
#endif

            //store
            xythetaPath->push_back(intermpt);
        }
    }
}

//returns the stateid if success, and -1 otherwise
int EnvironmentOMPL::SetGoal(double x_m, double y_m, double theta_rad)
{
    int x = CONTXY2DISC(x_m, EnvOMPLCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvOMPLCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvOMPLCfg.NumThetaDirs);

    SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    if (!IsValidConfiguration(x, y, theta)) {
        SBPL_PRINTF("WARNING: goal configuration is invalid\n");
    }

    EnvOMPLHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }

    //need to recompute start heuristics?
    if (EnvOMPL.goalstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
    }

    EnvOMPL.goalstateid = OutHashEntry->stateID;

    EnvOMPLCfg.EndX_c = x;
    EnvOMPLCfg.EndY_c = y;
    EnvOMPLCfg.EndTheta = theta;

    return EnvOMPL.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentOMPL::SetStart(double x_m, double y_m, double theta_rad)
{
    int x = CONTXY2DISC(x_m, EnvOMPLCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvOMPLCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvOMPLCfg.NumThetaDirs);

    if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    SBPL_PRINTF("env: setting start to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if (!IsValidConfiguration(x, y, theta)) {
        SBPL_PRINTF("WARNING: start configuration %d %d %d is invalid\n", x, y, theta);
    }

    EnvOMPLHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }

    //need to recompute start heuristics?
    if (EnvOMPL.startstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true;
        //because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true; 
    }

    //set start
    EnvOMPL.startstateid = OutHashEntry->stateID;
    EnvOMPLCfg.StartX_c = x;
    EnvOMPLCfg.StartY_c = y;
    EnvOMPLCfg.StartTheta = theta;

    return EnvOMPL.startstateid;
}

void EnvironmentOMPL::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if(stateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvOMPL... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if (stateID == EnvOMPL.goalstateid && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if (bVerbose)
        SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
    else
        SBPL_FPRINTF(fOut, "%.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, EnvOMPLCfg.cellsize_m),
                     DISCXY2CONT(HashEntry->Y, EnvOMPLCfg.cellsize_m),
                     DiscTheta2Cont(HashEntry->Theta, EnvOMPLCfg.NumThetaDirs));
}

EnvOMPLHashEntry_t* EnvironmentOMPL::GetHashEntry_lookup(int X, int Y, int Theta)
{
    if (X < 0 || X >= EnvOMPLCfg.EnvWidth_c || Y < 0 || Y >= EnvOMPLCfg.EnvHeight_c || Theta < 0 ||
        Theta >= EnvOMPLCfg.NumThetaDirs) return NULL;
    int index = XYTHETA2INDEX(X,Y,Theta);
    return Coord2StateIDHashTable_lookup[index];
}

EnvOMPLHashEntry_t* EnvironmentOMPL::GetHashEntry_hash(int X, int Y, int Theta)
{
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int binid = GETHASHBIN(X, Y, Theta);

#if DEBUG
    if ((int)Coord2StateIDHashTable[binid].size() > 5)
    {
        SBPL_FPRINTF(fDeb, "WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n",
            binid, X, Y, (int)Coord2StateIDHashTable[binid].size());

        PrintHashTableHist(fDeb);
    }
#endif

    //iterate over the states in the bin and select the perfect match
    vector<EnvOMPLHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
    for (int ind = 0; ind < (int)binV->size(); ind++) {
        EnvOMPLHashEntry_t* hashentry = binV->at(ind);
        if (hashentry->X == X && hashentry->Y == Y && hashentry->Theta == Theta) {
#if TIME_DEBUG
            time_gethash += clock()-currenttime;
#endif
            return hashentry;
        }
    }

#if TIME_DEBUG  
    time_gethash += clock()-currenttime;
#endif

    return NULL;
}

EnvOMPLHashEntry_t* EnvironmentOMPL::CreateNewHashEntry_lookup(int X, int Y, int Theta)
{
    int i;

#if TIME_DEBUG  
    clock_t currenttime = clock();
#endif

    EnvOMPLHashEntry_t* HashEntry = new EnvOMPLHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);

    int index = XYTHETA2INDEX(X,Y,Theta);

#if DEBUG
    if(Coord2StateIDHashTable_lookup[index] != NULL)
    {
        SBPL_ERROR("ERROR: creating hash entry for non-NULL hashentry\n");
        throw new SBPL_Exception();
    }
#endif
    //printf("index is %d\n",index);
    //printf("X = %d, Y = %d, Theta = %d\n",X,Y,Theta);
    Coord2StateIDHashTable_lookup[index] = HashEntry;

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }
    //printf("what abt here\n");
    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

EnvOMPLHashEntry_t* EnvironmentOMPL::CreateNewHashEntry_hash(int X, int Y, int Theta)
{
    int i;

#if TIME_DEBUG  
    clock_t currenttime = clock();
#endif

    EnvOMPLHashEntry_t* HashEntry = new EnvOMPLHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);

    //get the hash table bin
    i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta);

    //insert the entry into the bin
    Coord2StateIDHashTable[i].push_back(HashEntry);

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

void EnvironmentOMPL::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<
    EnvOMPLAction_t*>* actionV /*=NULL*/)
{

    //printf("Getting successors for %d\n", SourceStateID);
    int aind;
    double ompl_cont_x, ompl_cont_y;

    visualization_msgs::MarkerArray display_succ;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvOMPLCfg.actionwidth);
    CostV->reserve(EnvOMPLCfg.actionwidth);
    if (actionV != NULL) {
        actionV->clear();
        actionV->reserve(EnvOMPLCfg.actionwidth);
    }

    //goal state should be absorbing
    if (SourceStateID == EnvOMPL.goalstateid) return;

    //get X, Y for the state
    EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    ompl_cont_x = DISCXY2CONT(HashEntry->X, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_x;
    ompl_cont_y = DISCXY2CONT(HashEntry->Y, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_y;
    
    coord_disc.push_back(ompl_cont_x);
    coord_disc.push_back(ompl_cont_y);

    // stateToVisualizationMarker(coord_disc, true, false);
    //ros::Duration(0.01).sleep();
    //marker_array_pub.publish(display_succ);
    
    //coord.clear();



    //iterate through actions
    for (aind = 0; aind < EnvOMPLCfg.actionwidth; aind++) {
        EnvOMPLAction_t* nav3daction = &EnvOMPLCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvOMPLCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY)) continue;

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST) continue;

        EnvOMPLHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);

        }

        // ompl_cont_x = DISCXY2CONT(newX, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_x;
        // ompl_cont_y = DISCXY2CONT(newY, EnvOMPLCfg.cellsize_m) + EnvOMPLCfg.volume_min_y;
    
        // coord.push_back(ompl_cont_x);
        // coord.push_back(ompl_cont_y);

        //stateToVisualizationMarker(coord, true);
        //ros::Duration(0.01).sleep();
        //marker_array_pub.publish(display_succ);
    
        //coord.clear();

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        if (actionV != NULL) actionV->push_back(nav3daction);
    }


#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

visualization_msgs::MarkerArray EnvironmentOMPL::seeinRviz(){
    return ma;
}

void EnvironmentOMPL::stateToVisualizationMarker(std::vector<double> coord, bool is_discrete_state, bool is_path){
  if (!use_visualization_) return;
  visualization_msgs::Marker state;
  state.header.frame_id = "map";
  state.header.stamp = ros::Time();
  state.id = rand();
  state.action = visualization_msgs::Marker::ADD;
  state.type = visualization_msgs::Marker::CUBE;
  state.scale.x = kCellSize*3;
  state.scale.y = kCellSize*3;
  state.scale.z = kCellSize*3;
  state.pose.position.x = coord[0];
  state.pose.position.y = coord[1];
  state.pose.orientation.w = 1.0;

  visualization_msgs::Marker line_strip;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.id = rand();
  line_strip.scale.x = 0.1;
  line_strip.header.frame_id = "map";

  visualization_msgs::Marker path_strip;
  path_strip.type = visualization_msgs::Marker::LINE_STRIP;
  path_strip.id = rand();
  path_strip.scale.x = 0.5;
  path_strip.header.frame_id = "map";

  if (is_discrete_state && !is_path){
  state.color.r = 1.0f;
  state.color.g = 0.0f;
  state.color.b = 0.0f;
  state.color.a = 1.0;

  state.pose.position.z = 2.5;

  geometry_msgs::Point point;
  point.x = coord[0];
  point.y = coord[1];
  point.z = 2.5;
  //printf("The child is %f, %f\n",coord[0],coord[1]);
  line_strip.points.push_back(point);
  point.x = coord[2];
  point.y = coord[3];
  point.z = 2.5;
  //printf("The parent is %f, %f\n",coord[2],coord[3]);
  line_strip.points.push_back(point);

  line_strip.color.r = 1.0f;
  line_strip.color.a = 1.0;
  ma.markers.push_back(line_strip);
  } 
  else if (!is_discrete_state && !is_path){
  state.color.r = 0.0f;
  state.color.g = 1.0f;
  state.    color.b = 0.0f;
  state.color.a = 1.0;

  state.pose.position.z = 5;

  geometry_msgs::Point point;
  point.x = coord[0];
  point.y = coord[1];
  point.z = 5;
  //printf("The child is %f, %f\n",coord[0],coord[1]);
  line_strip.points.push_back(point);
  point.x = coord[2];
  point.y = coord[3];
  point.z = 5;
  //printf("The parent is %f, %f\n",coord[2],coord[3]);
  line_strip.points.push_back(point);

  line_strip.color.g = 1.0f;
  line_strip.color.a = 1.0;
  ma.markers.push_back(line_strip);
  }
  else{
  state.color.r = 0.0f;
  state.color.g = 0.0f;
  state.color.b = 0.0f;
  state.color.a = 1.0;

  state.pose.position.z = 7.5;

  geometry_msgs::Point point;
  point.x = coord[0];
  point.y = coord[1];
  point.z = 7.5;
  //printf("The child is %f, %f\n",coord[0],coord[1]);
  path_strip.points.push_back(point);
  point.x = coord[2];
  point.y = coord[3];
  point.z = 7.5;
  //printf("The parent is %f, %f\n",coord[2],coord[3]);
  path_strip.points.push_back(point);
  path_strip.color.r = 0.0f;
  path_strip.color.g = 0.0f;
  path_strip.color.b = 0.0f;
  path_strip.color.a = 1.0;
  ma.markers.push_back(path_strip);
  }
  state.lifetime = ros::Duration();

  ma.markers.push_back(state);
  //return ma;
}

void EnvironmentOMPL::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
    //TODO- to support tolerance, need:
    // a) generate preds for goal state based on all possible goal state variable settings,
    // b) change goal check condition in gethashentry c) change
    //    getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //get X, Y for the state
    EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvOMPLCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());
    CostV->reserve(EnvOMPLCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());

    //iterate through actions
    vector<EnvOMPLAction_t*>* actionsV = &EnvOMPLCfg.PredActionsV[(unsigned int)HashEntry->Theta];
    for (aind = 0; aind < (int)EnvOMPLCfg.PredActionsV[(unsigned int)HashEntry->Theta].size(); aind++) {

        EnvOMPLAction_t* nav3daction = actionsV->at(aind);

        int predX = HashEntry->X - nav3daction->dX;
        int predY = HashEntry->Y - nav3daction->dY;
        int predTheta = nav3daction->starttheta;

        //skip the invalid cells
        if (!IsValidCell(predX, predY)) continue;

        //get cost
        int cost = GetActionCost(predX, predY, predTheta, nav3daction);
        if (cost >= INFINITECOST) continue;

        EnvOMPLHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta);
        }

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void EnvironmentOMPL::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    int cost;

#if DEBUG
    if(state->StateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
        throw new SBPL_Exception();
    }                       

    if((int)state->Actions.size() != 0)
    {
        SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
        throw new SBPL_Exception();
    }
#endif

    //goal state should be absorbing
    if (state->StateID == EnvOMPL.goalstateid) return;

    //get X, Y for the state
    EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];

    //iterate through actions
    for (int aind = 0; aind < EnvOMPLCfg.actionwidth; aind++) {
        EnvOMPLAction_t* nav3daction = &EnvOMPLCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvOMPLCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY)) continue;

        //get cost
        cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST) continue;

        //add the action
        CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
        clock_t currenttime = clock();
#endif

        EnvOMPLHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }
        action->AddOutcome(OutHashEntry->stateID, cost, 1.0);

#if TIME_DEBUG
        time3_addallout += clock()-currenttime;
#endif
    }
}

void EnvironmentOMPL::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV,
                                                      vector<int> *preds_of_changededgesIDV)
{
    nav2dcell_t cell;
    sbpl_xy_theta_cell_t affectedcell;
    EnvOMPLHashEntry_t* affectedHashEntry;

    //increment iteration for processing savings
    iteration++;

    for (int i = 0; i < (int)changedcellsV->size(); i++) {
        cell = changedcellsV->at(i);

        //now iterate over all states that could potentially be affected
        for (int sind = 0; sind < (int)affectedpredstatesV.size(); sind++) {
            affectedcell = affectedpredstatesV.at(sind);

            //translate to correct for the offset
            affectedcell.x = affectedcell.x + cell.x;
            affectedcell.y = affectedcell.y + cell.y;

            //insert only if it was actually generated
            affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta);
            if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration) {
                preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                affectedHashEntry->iteration = iteration; //mark as already inserted
            }
        }
    }
}

void EnvironmentOMPL::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV,
                                                      vector<int> *succs_of_changededgesIDV)
{
    nav2dcell_t cell;
    sbpl_xy_theta_cell_t affectedcell;
    EnvOMPLHashEntry_t* affectedHashEntry;

    SBPL_ERROR("ERROR: getsuccs is not supported currently\n");
    throw new SBPL_Exception();

    //increment iteration for processing savings
    iteration++;

    //TODO - check
    for (int i = 0; i < (int)changedcellsV->size(); i++) {
        cell = changedcellsV->at(i);

        //now iterate over all states that could potentially be affected
        for (int sind = 0; sind < (int)affectedsuccstatesV.size(); sind++) {
            affectedcell = affectedsuccstatesV.at(sind);

            //translate to correct for the offset
            affectedcell.x = affectedcell.x + cell.x;
            affectedcell.y = affectedcell.y + cell.y;

            //insert only if it was actually generated
            affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta);
            if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration) {
                succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                affectedHashEntry->iteration = iteration; //mark as already inserted
            }
        }
    }
}

void EnvironmentOMPL::InitializeEnvironment()
{
    EnvOMPLHashEntry_t* HashEntry;

    int maxsize = EnvOMPLCfg.EnvWidth_c * EnvOMPLCfg.EnvHeight_c * EnvOMPLCfg.NumThetaDirs;

    if (maxsize <= SBPL_OMPL_MAXSTATESFORLOOKUP) {
        SBPL_PRINTF("environment stores states in lookup table\n");

        Coord2StateIDHashTable_lookup = new EnvOMPLHashEntry_t*[maxsize];
        for (int i = 0; i < maxsize; i++)
            Coord2StateIDHashTable_lookup[i] = NULL;
        GetHashEntry = &EnvironmentOMPL::GetHashEntry_lookup;
        CreateNewHashEntry = &EnvironmentOMPL::CreateNewHashEntry_lookup;

        //not using hash table
        HashTableSize = 0;
        Coord2StateIDHashTable = NULL;
    }
    else {
        SBPL_PRINTF("environment stores states in hashtable\n");

        //initialize the map from Coord to StateID
        HashTableSize = 4 * 1024 * 1024; //should be power of two
        Coord2StateIDHashTable = new vector<EnvOMPLHashEntry_t*> [HashTableSize];
        GetHashEntry = &EnvironmentOMPL::GetHashEntry_hash;
        CreateNewHashEntry = &EnvironmentOMPL::CreateNewHashEntry_hash;

        //not using hash
        Coord2StateIDHashTable_lookup = NULL;
    }

    //initialize the map from StateID to Coord
    StateID2CoordTable.clear();

    //create start state
    if ((HashEntry = (this->*GetHashEntry)(EnvOMPLCfg.StartX_c, EnvOMPLCfg.StartY_c,
                                           EnvOMPLCfg.StartTheta)) == NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvOMPLCfg.StartX_c, EnvOMPLCfg.StartY_c,
                                                EnvOMPLCfg.StartTheta);
    }
    EnvOMPL.startstateid = HashEntry->stateID;

    //create goal state
    if ((HashEntry = (this->*GetHashEntry)(EnvOMPLCfg.EndX_c, EnvOMPLCfg.EndY_c,
                                           EnvOMPLCfg.EndTheta)) == NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvOMPLCfg.EndX_c, EnvOMPLCfg.EndY_c,
                                                EnvOMPLCfg.EndTheta);
    }
    EnvOMPL.goalstateid = HashEntry->stateID;

    //initialized
    EnvOMPL.bInitialized = true;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentOMPL::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta)
{
    return inthash(inthash(X1) + (inthash(X2) << 1) + (inthash(Theta) << 2)) & (HashTableSize - 1);
}

void EnvironmentOMPL::PrintHashTableHist(FILE* fOut)
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < HashTableSize; j++) {
        if ((int)Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)Coord2StateIDHashTable[j].size() < 5)
            s1++;
        else if ((int)Coord2StateIDHashTable[j].size() < 25)
            s50++;
        else if ((int)Coord2StateIDHashTable[j].size() < 50)
            s100++;
        else if ((int)Coord2StateIDHashTable[j].size() < 100)
            s200++;
        else if ((int)Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_FPRINTF(fOut, "hash table histogram: 0:%d, <5:%d, <25:%d, <50:%d, <100:%d, <400:%d, >400:%d\n", s0, s1, s50,
                 s100, s200, s300, slarge);
}

int EnvironmentOMPL::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)StateID2CoordTable.size()
        || ToStateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvOMPL... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the state
    EnvOMPLHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvOMPLHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    //TODO - check if one of the gridsearches already computed and then use it.

    return (int)(OMPL_COSTMULT_MTOMM * EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X,
                                                                    ToHashEntry->Y) /
                 EnvOMPLCfg.nominalvel_mpersecs);

}

int EnvironmentOMPL::GetGoalHeuristic(int stateID)
{

#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvOMPL... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //printf("The stateID is %d\n", stateID);fflush(stdout);

    EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    //printf("getting heuristic for state %d %d\n", HashEntry->X, HashEntry->Y);fflush(stdout);
    //if (!IsValidCell(HashEntry->X, HashEntry->Y)) {
    //  return INFINITECOST;
    //}
    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); 

    int hEuclid = (int)(OMPL_COSTMULT_MTOMM * EuclideanDistance_m(HashEntry->X, HashEntry->Y,
                                                                           EnvOMPLCfg.EndX_c,
                                                                           EnvOMPLCfg.EndY_c));

   //hEuclid = 0;
    //h2D = 0;
    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D, hEuclid)) / EnvOMPLCfg.nominalvel_mpersecs);
}

int EnvironmentOMPL::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvOMPL... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
    int hEuclid = (int)(OMPL_COSTMULT_MTOMM * EuclideanDistance_m(EnvOMPLCfg.StartX_c,
                                                                           EnvOMPLCfg.StartY_c, HashEntry->X,
                                                                           HashEntry->Y));

    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D, hEuclid)) / EnvOMPLCfg.nominalvel_mpersecs);
}

int EnvironmentOMPL::SizeofCreatedEnv()
{
    return (int)StateID2CoordTable.size();
}

const EnvOMPLHashEntry_t*
EnvironmentOMPL::GetStateEntry(int state_id) const
{
    if (state_id >= 0 && state_id < (int)StateID2CoordTable.size()) {
        return StateID2CoordTable[state_id];
    }
    else {
        return NULL;
    }
}

//------------------------------------------------------------------------------


void EnvironmentOMPL::GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost,
                                        std::vector<EnvOMPLAction_t*>* actionV /*=NULL*/)
{
  ///  TODO: change!!!
  GetSuccs(SourceStateID, SuccIDV, CostV, actionV);
  isTrueCost->resize(SuccIDV->size(), true);
  return;

    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvOMPLCfg.actionwidth);
    CostV->reserve(EnvOMPLCfg.actionwidth);
    isTrueCost->reserve(EnvOMPLCfg.actionwidth);
    if (actionV != NULL) {
        actionV->clear();
        actionV->reserve(EnvOMPLCfg.actionwidth);
    }

    //goal state should be absorbing
    if (SourceStateID == EnvOMPL.goalstateid) return;

    //get X, Y for the state
    EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    //iterate through actions
    for (aind = 0; aind < EnvOMPLCfg.actionwidth; aind++) {
        EnvOMPLAction_t* nav3daction = &EnvOMPLCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvOMPLCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY)) continue;

        if(!actionV){//if we are supposed to return the action, then don't do lazy
          EnvOMPLHashEntry_t* OutHashEntry;
          if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
          SuccIDV->push_back(OutHashEntry->stateID);
          CostV->push_back(nav3daction->cost);
          isTrueCost->push_back(false);
          continue;
        }

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST) continue;

        EnvOMPLHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        isTrueCost->push_back(true);
        if (actionV != NULL) actionV->push_back(nav3daction);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

int EnvironmentOMPL::GetTrueCost(int parentID, int childID){
  EnvOMPLHashEntry_t* fromHash = StateID2CoordTable[parentID];
  EnvOMPLHashEntry_t* toHash = StateID2CoordTable[childID];

  for(int i=0; i<EnvOMPLCfg.actionwidth; i++){
    EnvOMPLAction_t* nav3daction = &EnvOMPLCfg.ActionsV[(unsigned int)fromHash->Theta][i];
    int newX = fromHash->X + nav3daction->dX;
    int newY = fromHash->Y + nav3daction->dY;
    int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvOMPLCfg.NumThetaDirs);

    //skip the invalid cells
    if(!IsValidCell(newX, newY))
      continue;

    EnvOMPLHashEntry_t* hash;
    if((hash = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
      continue;
    if(hash->stateID != toHash->stateID)
      continue;

    //get cost
    int cost = GetActionCost(fromHash->X, fromHash->Y, fromHash->Theta, nav3daction);

    if(cost >= INFINITECOST)
      return -1;
    return cost;
  }
  printf("this should never happen! we didn't find the state we need to get the true cost for!\n");
  throw new SBPL_Exception();
  return -1;
}

void EnvironmentOMPL::GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                                                     std::vector<EnvOMPLAction_t*>* actionV /*=NULL*/){
  GetSuccs(SourceStateID, SuccIDV, CostV, actionV);
}

void EnvironmentOMPL::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost,
                                                     std::vector<EnvOMPLAction_t*>* actionV /*=NULL*/){
  GetLazySuccs(SourceStateID, SuccIDV, CostV, isTrueCost, actionV);
}

bool EnvironmentOMPL::isGoal(int id){
  return EnvOMPL.goalstateid == id;
}

void EnvironmentOMPL::GetLazyPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV, vector<bool>* isTrueCost)
{
  int aind;

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  //get X, Y for the state
  EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

  //clear the successor array
  PredIDV->clear();
  CostV->clear();
  PredIDV->reserve(EnvOMPLCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());
  CostV->reserve(EnvOMPLCfg.PredActionsV[(unsigned int)HashEntry->Theta].size());

  //iterate through actions
  vector<EnvOMPLAction_t*>* actionsV = &EnvOMPLCfg.PredActionsV[(unsigned int)HashEntry->Theta];
  for (aind = 0; aind < (int)EnvOMPLCfg.PredActionsV[(unsigned int)HashEntry->Theta].size(); aind++) {

    EnvOMPLAction_t* nav3daction = actionsV->at(aind);

    int predX = HashEntry->X - nav3daction->dX;
    int predY = HashEntry->Y - nav3daction->dY;
    int predTheta = nav3daction->starttheta;

    //skip the invalid cells
    if (!IsValidCell(predX, predY)) continue;

    EnvOMPLHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta)) == NULL)
      OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta);
    PredIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(nav3daction->cost);
    isTrueCost->push_back(false);
  }

#if TIME_DEBUG
  time_getsuccs += clock()-currenttime;
#endif
}

void EnvironmentOMPL::GetPredsWithUniqueIds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV){
  GetPreds(TargetStateID, PredIDV, CostV);
}

void EnvironmentOMPL::GetLazyPredsWithUniqueIds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV, vector<bool>* isTrueCost){
  GetLazyPreds(TargetStateID, PredIDV, CostV, isTrueCost);
}

std::string trim(std::string const& source, char const* delims = " \t\r\n") {
  std::string result(source);
  std::string::size_type index = result.find_last_not_of(delims);
  if(index != std::string::npos)
    result.erase(++index);

  index = result.find_first_not_of(delims);
  if(index != std::string::npos)
    result.erase(0, index);
  else
    result.erase();
  return result;
}


void EnvironmentOMPL::GetNearestLatticeState(const ompl::base::State *continuous_state, ompl::base::State* nearest_lattice_state, int *nearest_lattice_state_id) {
  const ompl::base::SE2StateSpace::StateType* se2_state = continuous_state->as<ompl::base::SE2StateSpace::StateType>();
  int disc_x = CONTXY2DISC(se2_state->getX() + EnvOMPLCfg.volume_max_x, EnvOMPLCfg.cellsize_m);
  int disc_y = CONTXY2DISC(se2_state->getY() + EnvOMPLCfg.volume_max_y, EnvOMPLCfg.cellsize_m);
  int disc_theta = ContTheta2Disc(normalizeAngle(se2_state->getYaw()), EnvOMPLCfg.NumThetaDirs);

  // printf("The continous valules from get nearest latttice are %f,%f and %f\n",se2_state->getX(),se2_state->getY(),se2_state->getYaw());
  // printf("The discrete valules are %d,%d and %d\n",disc_x,disc_y,disc_theta);

  EnvOMPLHashEntry_t* HashEntry;
  if ((HashEntry = (this->*GetHashEntry)(disc_x, disc_y, disc_theta)) == NULL) {
      //have to create a new entry
      HashEntry = (this->*CreateNewHashEntry)(disc_x, disc_y, disc_theta);
  }
  *nearest_lattice_state_id = HashEntry->stateID;

  double cont_x = DISCXY2CONT(disc_x, EnvOMPLCfg.cellsize_m);
  double cont_y = DISCXY2CONT(disc_y, EnvOMPLCfg.cellsize_m);
  double cont_theta = DiscTheta2Cont(disc_theta, EnvOMPLCfg.NumThetaDirs);
  
  //printf("The continous valules are %f,%f and %f\n",cont_x,cont_y,cont_theta);

  nearest_lattice_state->as<ompl::base::SE2StateSpace::StateType>()->setX(cont_x + EnvOMPLCfg.volume_min_x);
  nearest_lattice_state->as<ompl::base::SE2StateSpace::StateType>()->setY(cont_y + EnvOMPLCfg.volume_min_y);
  nearest_lattice_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(angles::normalize_angle(cont_theta));
}

void EnvironmentOMPL::VisualizeContState(const ompl::base::State *state, const ompl::base::State *parent, bool is_discrete, bool is_path) {
  //return;
  vector<double> coord = {state->as<ompl::base::SE2StateSpace::StateType>()->getX(), state->as<ompl::base::SE2StateSpace::StateType>()->getY(), 
  parent->as<ompl::base::SE2StateSpace::StateType>()->getX(),parent->as<ompl::base::SE2StateSpace::StateType>()->getY()};
  stateToVisualizationMarker(coord, is_discrete, is_path);
  //ros::Duration(0.01).sleep();
  //marker_array_pub.publish(display_succ);
}

// Get the continuous-state equivalent for a lattice state.
void EnvironmentOMPL::GetContState(int state_id, ompl::base::State *state) {
  EnvOMPLHashEntry_t* HashEntry = StateID2CoordTable[state_id];

  int disc_x = HashEntry->X;
  int disc_y = HashEntry->Y;
  int disc_theta = HashEntry->Theta;

  double cont_x = DISCXY2CONT(disc_x, EnvOMPLCfg.cellsize_m);
  double cont_y = DISCXY2CONT(disc_y, EnvOMPLCfg.cellsize_m);
  double cont_theta = DiscTheta2Cont(disc_theta, EnvOMPLCfg.NumThetaDirs);

  state->as<ompl::base::SE2StateSpace::StateType>()->setX(cont_x + EnvOMPLCfg.volume_min_x);
  state->as<ompl::base::SE2StateSpace::StateType>()->setY(cont_y + EnvOMPLCfg.volume_min_y);
  state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(cont_theta);
}

int EnvironmentOMPL::GetContStateID(const ompl::base::State* state) {
  // printf("The Continous hash counter is %d\n",cont_hash_counter);fflush(stdout);

  // int disc_x = CONTXY2DISC(EnvOMPLCfg.volume_max_x - EnvOMPLCfg.volume_min_x, EnvOMPLCfg.cellsize_m);
  // int disc_y = CONTXY2DISC(EnvOMPLCfg.volume_max_y - EnvOMPLCfg.volume_min_y, EnvOMPLCfg.cellsize_m);
  // int disc_theta = ContTheta2Disc(2*M_PI - M_PI/16, EnvOMPLCfg.NumThetaDirs);

  // printf("The discrete valules are %d,%d and %d\n",disc_x,disc_y,disc_theta);fflush(stdout);

  // EnvOMPLHashEntry_t* HashEntry;  
  // if ((HashEntry = (this->*GetHashEntry)(disc_x, disc_y, disc_theta)) == NULL) {
  //     HashEntry = (this->*CreateNewHashEntry)(disc_x, disc_y, disc_theta);
  // }
  int cont_state_id = cont_hash_counter;
  cont_hash_counter++;
  if (cont_hash_counter < SizeofCreatedEnv()) {
    printf("We have a ID problem!!!!!!!!\n");
    exit(1);
  }

  //printf("The continous StateID is %d\n", cont_state_id);fflush(stdout);

  return cont_state_id;
}

int EnvironmentOMPL::GetContEdgeCost(const ompl::base::State *parent_state, const ompl::base::State *child_state) {
  // TODO: this should use the same optimization objective used by the OMPL
  // benchmarks. Also, GetActionCost should call this method to compute the cost
  // of an edge.
  // printf("Hi..................................\n");fflush(stdout);
  //const auto &objective = *cost_obj;
  ompl::base::Cost contEdgeCost = cost_obj->motionCost(parent_state, child_state);
  double x = parent_state->as<ompl::base::SE2StateSpace::StateType>()->getX();
  double y = parent_state->as<ompl::base::SE2StateSpace::StateType>()->getY();
  double theta = parent_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
  //printf("The x,y and theta of parent state is %f,%f and %f\n",x,y,theta);

  x = child_state->as<ompl::base::SE2StateSpace::StateType>()->getX();
  y = child_state->as<ompl::base::SE2StateSpace::StateType>()->getY();
  theta = child_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
  //printf("The x,y and theta of child state is %f,%f and %f\n",x,y,theta);

  int int_contEdgeCost = (int)(OMPL_COSTMULT_MTOMM*(contEdgeCost.value()) ); 

  // /printf("The edgecost is %d\n",int_contEdgeCost);

  return int_contEdgeCost;
}

myMotionValidator::myMotionValidator(const ompl::base::SpaceInformationPtr &si_) : ompl::base::MotionValidator(si_){

}

bool myMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair< ompl::base::State *, double > &lastValid) const
{    
    double edge_length = si_->distance(s1, s2);
    int num_segments = static_cast<int>(edge_length / kCellSize);
    double interp_factor = 1.0 / static_cast<double>(num_segments);
    bool valid = true;
    ompl::base::State *interp_state = si_->allocState();
    for (double t = 0 ; t <=1; t+=interp_factor) {
        si_->getStateSpace()->interpolate(s1, s2, t, interp_state);
        if (!si_->isValid(interp_state)) {
        valid = false;
        break;
        }
    }
  
    si_->freeState(interp_state);
  
    return valid;
}

ConfigFile::ConfigFile(std::string const& configFile){
  std::ifstream file(configFile.c_str());

  std::string line;
  std::string name;
  std::string value;
  std::string inSection;
  int posEqual;
  while (std::getline(file,line)) {

    if (! line.length()) continue;

    if (line[0] == '#') continue;
    if (line[0] == ';') continue;

    if (line[0] == '[') {
      inSection=trim(line.substr(1,line.find(']')-1));
      continue;
    }

    posEqual=line.find('=');
    name  = trim(line.substr(0,posEqual));
    value = trim(line.substr(posEqual+1));

    content_[inSection+'/'+name]=Chameleon(value);
  }
}


Chameleon const& ConfigFile::Value(std::string const& section, std::string const& entry) const {

  std::map<std::string,Chameleon>::const_iterator ci = content_.find(section + '/' + entry);

  if (ci == content_.end()) throw "does not exist";

  return ci->second;
}

Chameleon const& ConfigFile::Value(std::string const& section, std::string const& entry, double value) {
  try {
    return Value(section, entry);
  } catch(const char *) {
    return content_.insert(std::make_pair(section+'/'+entry, Chameleon(value))).first->second;
  }
}

Chameleon const& ConfigFile::Value(std::string const& section, std::string const& entry, std::string const& value) {
  try {
    return Value(section, entry);
  } catch(const char *) {
    return content_.insert(std::make_pair(section+'/'+entry, Chameleon(value))).first->second;
  }
}
