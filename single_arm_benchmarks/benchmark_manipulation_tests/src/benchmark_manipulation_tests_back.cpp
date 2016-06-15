#include <benchmark_manipulation_tests/benchmark_manipulation_tests.h>
#include <sys/stat.h>
#include <dirent.h>

#define PLANNER_TO_SET_CURRENT_POSE 0

bool first_run;

BenchmarkManipulationTests::BenchmarkManipulationTests() : ph_("~")
{
  action_list_.push_back("nothing");
  action_list_.push_back("attach");
  action_list_.push_back("detach");
  action_map_["nothing"] = 0;
  action_map_["attach"] = 1;
  action_map_["detach"] = 2;
  group_name_ = "right_arm";
  planner_id_ = "sbpl_arm_planner";
  experiment_type_ = 1;
  num_joints_ = 7;
  trajectory_folder_path_ = "/tmp";
  use_current_state_as_start_ = true;
  use_initial_start_as_every_start_ = false;

  rjoint_names_.push_back("r_shoulder_pan_joint");
  rjoint_names_.push_back("r_shoulder_lift_joint");
  rjoint_names_.push_back("r_upper_arm_roll_joint");
  rjoint_names_.push_back("r_elbow_flex_joint");
  rjoint_names_.push_back("r_forearm_roll_joint");
  rjoint_names_.push_back("r_wrist_flex_joint");
  rjoint_names_.push_back("r_wrist_roll_joint");
  ljoint_names_.push_back("l_shoulder_pan_joint");
  ljoint_names_.push_back("l_shoulder_lift_joint");
  ljoint_names_.push_back("l_upper_arm_roll_joint");
  ljoint_names_.push_back("l_elbow_flex_joint");
  ljoint_names_.push_back("l_forearm_roll_joint");
  ljoint_names_.push_back("l_wrist_flex_joint");
  ljoint_names_.push_back("l_wrist_roll_joint");

  fk_link_.resize(2);
  fk_link_[0] = "r_elbow_flex_link";
  fk_link_[1] = "r_wrist_roll_link";

  benchmark_service_name_="/benchmark_planning_problem";
  world_frame_="base_footprint";
  robot_model_root_frame_="odom_combined";
  spine_frame_="torso_lift_link";

  pviz_.setReferenceFrame("base_footprint");
  pscene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
  first_run = true;

  ma_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
}

bool BenchmarkManipulationTests::getParams()
{
  
  req2.body_start.resize(4);
  req2.rarm_start.resize(7);
  req2.larm_start.resize(7);
  req2.body_goal.resize(4);
  req2.rarm_goal.resize(7);
  req2.larm_goal.resize(7);
  // printf("rarmmmmmm start %f %f %f %f %f %f %f\n", req2.rarm_start[0], req2.rarm_start[1], req2.rarm_start[2],req2.rarm_start[3],req2.rarm_start[4],
  //   req2.rarm_start[5],req2.rarm_start[6]);
  // printf("rarmmmmmm goal %f %f %f %f %f %f %f\n", req2.rarm_goal[0], req2.rarm_goal[1], req2.rarm_goal[2],req2.rarm_goal[3],req2.rarm_goal[4],
  //   req2.rarm_goal[5],req2.rarm_goal[6]);

  XmlRpc::XmlRpcValue plist;
  std::string p;

  ph_.param<std::string>("known_objects_filename",known_objects_filename_, "");
  ph_.param<std::string>("ompl_planner_id",ompl_planner_id_, "");
  ph_.param<std::string>("attached_object_filename",attached_object_filename_, "");
  ph_.param<std::string>("trajectory_folder_path",trajectory_folder_path_, "/tmp");
  ph_.param("apply_offset_to_collision_objects",apply_offset_to_collision_objects_,false);

  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));;
  time.erase(time.size()-1, 1);
  trajectory_files_path_ = trajectory_folder_path_ + "/" + time;
  if(mkdir(trajectory_files_path_.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)
    ROS_INFO("Successfully created the trajectory folder: %s", trajectory_files_path_.c_str());
  else
    ROS_WARN("Failed to create the trajectory folder: %s", trajectory_files_path_.c_str());
  
  if(ph_.hasParam("goal_tolerance/xyz") && ph_.hasParam("goal_tolerance/rpy"))
  {
    ph_.getParam("goal_tolerance/xyz", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      goal_tolerance_.push_back(atof(p.c_str()));

    ph_.getParam("goal_tolerance/rpy", plist);
    std::stringstream ss1(plist);
    while(ss1 >> p)
      goal_tolerance_.push_back(atof(p.c_str()));
  }
  else
  {
    goal_tolerance_.resize(6,0.02);
    goal_tolerance_[3] = 0.05;
    goal_tolerance_[4] = 0.05;
    goal_tolerance_[5] = 0.05;
  }

  ph_.param<std::string>("benchmark_service", benchmark_service_name_, "/benchmark_planning_problem"); 
  ph_.param<std::string>("world_frame", world_frame_, "base_footprint");
  ph_.param<std::string>("robot_model_root_frame", robot_model_root_frame_, "odom_combined");
  ph_.param<std::string>("spine_frame", spine_frame_, "torso_lift_joint");
  ph_.param<std::string>("benchmark_results_folder", benchmark_results_folder_, "/tmp");
  ph_.param<std::string>("experiment_group_name", experiment_group_name_, "no_group_name");
  ph_.param<std::string>("trajectory_description_for_display", display_trajectory_description_, "plan");
  ph_.param("average_count", average_count_, 2);
  
  if(ph_.hasParam("object_pose_in_gripper"))
  {
    rarm_object_offset_.clear();
    larm_object_offset_.clear();
    ph_.getParam("object_pose_in_gripper/right/xyz", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      rarm_object_offset_.push_back(atof(p.c_str()));

    ph_.getParam("object_pose_in_gripper/right/rpy", plist); 
    std::stringstream ss1(plist);
    while(ss1 >> p)
      rarm_object_offset_.push_back(atof(p.c_str()));
    
    ph_.getParam("object_pose_in_gripper/left/xyz", plist);
    std::stringstream ss2(plist);
    while(ss2 >> p)
      larm_object_offset_.push_back(atof(p.c_str()));

    ph_.getParam("object_pose_in_gripper/left/rpy", plist); 
    std::stringstream ss3(plist);
    while(ss3 >> p)
      larm_object_offset_.push_back(atof(p.c_str()));
  }
 
  if(ph_.hasParam("collision_object_offset"))
  {
    collision_object_offset_.clear();
    ph_.getParam("collision_object_offset/xyz", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      collision_object_offset_.push_back(atof(p.c_str()));

    ph_.getParam("collision_object_offset/rpy", plist);
    std::stringstream ss1(plist);
    while(ss1 >> p)
      collision_object_offset_.push_back(atof(p.c_str()));

    tf::Quaternion btoffset;
    collision_object_offset_pose_.position.x = collision_object_offset_[0];
    collision_object_offset_pose_.position.y = collision_object_offset_[1];
    collision_object_offset_pose_.position.z = collision_object_offset_[2];
    btoffset.setRPY(collision_object_offset_[3],collision_object_offset_[4],collision_object_offset_[5]);
    tf::quaternionTFToMsg(btoffset,collision_object_offset_pose_.orientation);
  }
  
  tf::Quaternion btoffset;
  rarm_object_pose_.position.x = rarm_object_offset_[0];
  rarm_object_pose_.position.y = rarm_object_offset_[1];
  rarm_object_pose_.position.z = rarm_object_offset_[2];
  larm_object_pose_.position.x = larm_object_offset_[0];
  larm_object_pose_.position.y = larm_object_offset_[1];
  larm_object_pose_.position.z = larm_object_offset_[2];
  btoffset.setRPY(rarm_object_offset_[3],rarm_object_offset_[4],rarm_object_offset_[5]);
  tf::quaternionTFToMsg(btoffset,rarm_object_pose_.orientation);
  btoffset.setRPY(larm_object_offset_[3],larm_object_offset_[4],larm_object_offset_[5]);
  tf::quaternionTFToMsg(btoffset,larm_object_pose_.orientation);

  if(ph_.hasParam("use_current_pose_as_start_state"))
    ph_.getParam("use_current_pose_as_start_state", use_current_state_as_start_);
 
  if(ph_.hasParam("use_initial_start_state_for_all_experiments"))
    ph_.getParam("use_initial_start_state_for_all_experiments", use_initial_start_as_every_start_);
 
  if(ph_.hasParam("group_name"))
  {
    ph_.getParam("group_name", group_name_);
    if(group_name_.compare("right_arm") == 0)
      experiment_type_ = 1;
    else
    {
      ROS_ERROR("[exp] This infrastructure only supports group_names of {'right_arm'}. Exiting.");
      return false;
    }
  }
  else
  {
    group_name_ = "right_arm";
    experiment_type_ = 1;
  }
  
  if(ph_.hasParam("initial_start_state"))
  {
    start_pose_.rangles.clear();
    start_pose_.langles.clear();
    std::vector<double> bpose;
    ph_.getParam("initial_start_state/right", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      start_pose_.rangles.push_back(atof(p.c_str()));
    ph_.getParam("initial_start_state/left", plist);
    std::stringstream ss1(plist);
    ss.str(plist);
    while(ss1 >> p)
      start_pose_.langles.push_back(atof(p.c_str()));
    ph_.getParam("initial_start_state/base", plist);
    std::stringstream ss2(plist);
    while(ss2 >> p)
      bpose.push_back(atof(p.c_str()));
    if(bpose.size() == 3)
    {
      start_pose_.body.x = bpose[0];
      start_pose_.body.y = bpose[1];
      start_pose_.body.theta = bpose[2];
    }
    ph_.getParam("initial_start_state/spine", start_pose_.body.z);  
  }

  start_pose_.rangles = req2.rarm_start;
  // start_pose_.body.z = req2.body_start[2];
  if(ph_.hasParam("planner_interfaces"))
  {
    ph_.getParam("planner_interfaces", plist);
    std::string planner_list = std::string(plist);
    std::stringstream ss(planner_list);
    while(ss >> p)
      planner_interfaces_.push_back(p);
  }

  if(goal_tolerance_.size() < 6 || 
      start_pose_.rangles.size() < 7 ||
      start_pose_.langles.size() < 7)
  {
    ROS_ERROR("[exp] Missing some params. Either start angles for the arms or the goal tolerance.");
    return false;
  }

  if((rarm_object_offset_.size() < 6 || larm_object_offset_.size() < 6) && experiment_type_ == 2)
    return false;


  if(!getLocations() || !getExperiments())
    return false;

  return true;
}

bool BenchmarkManipulationTests::getLocations()
{
  XmlRpc::XmlRpcValue loc_list;
  geometry_msgs::Pose p;
  std::string name;
  std::string loc_name = "locations";

  if(!ph_.hasParam(loc_name))
  {
    ROS_WARN("[exp] No list of locations found on the param server.");
    return false;
  }
  ph_.getParam(loc_name, loc_list);

  if(loc_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("[exp] Location list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(loc_list.size() == 0)
  {
    ROS_ERROR("[exp] List of locations is empty.");
    return false;
  }

  for(int i = 0; i < loc_list.size(); ++i)
  {
    if(!loc_list[i].hasMember("name"))
    {
      ROS_ERROR("Each location must have a name.");
      return false;
    }
    name = std::string(loc_list[i]["name"]);
    std::stringstream ss(loc_list[i]["pose"]);
    std::string p;
    while(ss >> p)
      loc_map_[name].push_back(atof(p.c_str()));
  }

  ROS_INFO("[exp] Successfully fetched %d locations from param server.", int(loc_list.size()));
  return true;
}

bool BenchmarkManipulationTests::getExperiments()
{
  XmlRpc::XmlRpcValue exp_list;
  Experiment e;
  std::string exp_name = "experiments";
  XmlRpc::XmlRpcValue plist;
  std::string p;

  if(!ph_.hasParam(exp_name))
  {
    ROS_WARN("No list of experiments found on the param server.");
    return false;
  }
  ph_.getParam(exp_name, exp_list);

  if(exp_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Experiment list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(exp_list.size() == 0)
  {
    ROS_ERROR("List of experiments is empty.");
    return false;
  }

  for(int i = 0; i < exp_list.size(); ++i)
  {
    if(!exp_list[i].hasMember("name"))
    {
      ROS_ERROR("Each experiment must have a name.");
      return false;
    }
    e.name = std::string(exp_list[i]["name"]);

    if(!exp_list[i].hasMember("goal"))
    {
      ROS_ERROR("Each experiment must have a goal....duh.");
      return false;
    }
    e.goal = std::string(exp_list[i]["goal"]);

    if(!exp_list[i].hasMember("pre_action"))
      e.pre_action = 0;
    else
      e.pre_action = action_map_[exp_list[i]["pre_action"]];

    if(!exp_list[i].hasMember("post_action"))
      e.post_action = 0;
    else
      e.post_action = action_map_[exp_list[i]["post_action"]];

    if(!exp_list[i].hasMember("sound_bite"))
      e.sound_bite = "";
    else
      e.sound_bite = std::string(exp_list[i]["sound_bite"]);

    if(exp_list[i].hasMember("start"))
    {
      e.start.rangles.clear();
      e.start.langles.clear();
      std::vector<double> bpose;
      plist = exp_list[i]["start"]["right"];
      std::stringstream ss(plist);
      while(ss >> p)
        e.start.rangles.push_back(atof(p.c_str()));
      
      plist = exp_list[i]["start"]["left"];
      std::stringstream ss1(plist);
      while(ss1 >> p)
        e.start.langles.push_back(atof(p.c_str()));
      
      plist = exp_list[i]["start"]["base"];
      std::stringstream ss2(plist);
      while(ss2 >> p)
        bpose.push_back(atof(p.c_str()));
      if(bpose.size() == 3)
      {
        e.start.body.x = bpose[0];
        e.start.body.y = bpose[1];
        e.start.body.theta = bpose[2];
      }
      e.start.body.z = double(exp_list[i]["start"]["spine"]);
    }
    else
    {
      if(!use_current_state_as_start_ && !use_initial_start_as_every_start_)
      {
        ROS_ERROR("[exp] No start state defined for %s and it isn't configured to use the current state as the start state.",e.name.c_str());
        return false; 
      }
      else  
        ROS_DEBUG("No start state defined for %s but it's OK because it's configured to use the current state as the start.",e.name.c_str());
    }

    ROS_DEBUG("Adding experiment: %s", e.name.c_str());
    exp_map_[e.name] = e;
  }

  return true;  
}

bool BenchmarkManipulationTests::getCollisionObjects(std::string filename, std::vector<arm_navigation_msgs::CollisionObject> &collision_objects)
{
  int num_obs;
  char sTemp[1024];
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  std::vector<std::vector<double> > objects, object_colors;
  std::vector<std::string> object_ids;
  arm_navigation_msgs::CollisionObject object;
  collision_objects.clear();

  FILE* fCfg = fopen(filename.c_str(), "r");
  if(fCfg == NULL)
    return false;

  // get number of objects
  if(fscanf(fCfg,"%d",&num_obs) < 1)
    ROS_INFO("[exp] Parsed string has length < 1.(number of obstacles)\n");

  ROS_INFO("[exp] Parsing collision object file with %i objects.",num_obs);

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs, std::vector<double>(6,0.0));
  object_colors.resize(num_obs, std::vector<double>(4,0.0));
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      ROS_INFO("[exp] Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[exp] Parsed string has length < 1. (object parameters for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
    for(int j=0; j < 4; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[exp] Parsed string has length < 1. (object colors for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        object_colors[i][j] = atof(sTemp);
    }
  }

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return false;
  }

  //pviz_.visualizeObstacles(objects);
  object.shapes.resize(1);
  object.poses.resize(1);
  object.shapes[0].dimensions.resize(3);
  //object.shapes[0].triangles.resize(4);
  for(size_t i = 0; i < objects.size(); i++)
  {
    object.id = object_ids[i];
    object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    object.header.frame_id = "base_footprint";
    object.header.stamp = ros::Time::now();

    object.poses[0].position.x = objects[i][0];
    object.poses[0].position.y = objects[i][1];
    object.poses[0].position.z = objects[i][2];
    object.poses[0].orientation.x = 0; 
    object.poses[0].orientation.y = 0; 
    object.poses[0].orientation.z = 0; 
    object.poses[0].orientation.w = 1;  

    object.shapes[0].dimensions[0] = objects[i][3];
    object.shapes[0].dimensions[1] = objects[i][4];
    object.shapes[0].dimensions[2] = objects[i][5];

    // apply collision object offset
    // right now just translates and rotates about z
    if(apply_offset_to_collision_objects_)
    {
      if(!collision_object_offset_.empty()) 
      {
        geometry_msgs::Pose p, p2;
        Eigen::Affine3d a;
        a(0,0) = cos(collision_object_offset_[5]);
        a(1,0) = sin(collision_object_offset_[5]);
        a(2,0) = 0;
        a(0,1) = -sin(collision_object_offset_[5]);
        a(1,1) = cos(collision_object_offset_[5]);
        a(2,1) = 0;
        a(0,2) = 0;
        a(1,2) = 0;
        a(2,2) = 0;
        leatherman::msgFromPose(a, p2);
        p2.position.x = 0;
        p2.position.y = 0;
        p2.position.z = 0;
        leatherman::multiplyPoses(p2, object.poses[0], p);
        p.position.x += collision_object_offset_pose_.position.x; 
        p.position.y += collision_object_offset_pose_.position.y; 
        p.position.z += collision_object_offset_pose_.position.z;
        object.poses[0] = p;
      }
      else
        ROS_ERROR("[exp] Expecting to translate/rotate collision objects in robot frame but offset not found.");
    }

    collision_objects.push_back(object);
    ROS_DEBUG("[exp] [%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f colors: %2.0f %2.0f %2.0f %2.0f",int(i),object_ids[i].c_str(),objects[i][0],objects[i][1],objects[i][2],objects[i][3],objects[i][4],objects[i][5], object_colors[i][0], object_colors[i][1], object_colors[i][2], object_colors[i][3]);
    // ROS_INFO("[exp] [%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f colors: %2.0f %2.0f %2.0f %2.0f",int(i),object_ids[i].c_str(),object.poses[0].position.x,object.poses[0].position.y,object.poses[0].position.z, object.shapes[0].dimensions[0], object.shapes[0].dimensions[1], object.shapes[0].dimensions[2], object_colors[i][0], object_colors[i][1], object_colors[i][2], object_colors[i][3]);

    std::vector<double> dim(3,0);
    dim[0] = objects[i][3];
    dim[1] = objects[i][4];
    dim[2] = objects[i][5];
    pviz_.getCubeMsg(object.poses[0], dim, object_colors[i], "collision_objects", int(i), marker);
    marker_array.markers.push_back(marker);
  }
  ROS_INFO("[exp] I gathered %d collision cubes", int(marker_array.markers.size()));
  pviz_.publishMarkerArray(marker_array);
  usleep(500);
  return true;
}

bool BenchmarkManipulationTests::getCollisionObjects(std::string filename, std::vector<moveit_msgs::CollisionObject> &collision_objects)
{
  int num_obs;
  char sTemp[1024];
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  std::vector<std::vector<double> > objects, object_colors;
  std::vector<std::string> object_ids;
  moveit_msgs::CollisionObject object;
  collision_objects.clear();

  FILE* fCfg = fopen(filename.c_str(), "r");
  if(fCfg == NULL)
    return false;

  // get number of objects
  if(fscanf(fCfg,"%d",&num_obs) < 1)
    ROS_INFO("[exp] Parsed string has length < 1.(number of obstacles)\n");

  ROS_INFO("[exp] Parsing collision object file with %i objects.",num_obs);

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs, std::vector<double>(6,0.0));
  object_colors.resize(num_obs, std::vector<double>(4,0.0));
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      ROS_INFO("[exp] Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[exp] Parsed string has length < 1. (object parameters for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
    for(int j=0; j < 4; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[exp] Parsed string has length < 1. (object colors for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        object_colors[i][j] = atof(sTemp);
    }
  }

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return false;
  }

  //pviz_.visualizeObstacles(objects);
  object.primitives.resize(1);
  object.primitive_poses.resize(1);
  object.primitives[0].dimensions.resize(3);
  //object.shapes[0].triangles.resize(4);
  for(size_t i = 0; i < objects.size(); i++)
  {
    object.id = object_ids[i];
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    object.header.frame_id = "base_footprint";
    object.header.stamp = ros::Time::now();

    object.primitive_poses[0].position.x = objects[i][0];
    object.primitive_poses[0].position.y = objects[i][1];
    object.primitive_poses[0].position.z = objects[i][2];
    object.primitive_poses[0].orientation.x = 0; 
    object.primitive_poses[0].orientation.y = 0; 
    object.primitive_poses[0].orientation.z = 0; 
    object.primitive_poses[0].orientation.w = 1;  

    object.primitives[0].dimensions[0] = objects[i][3];
    object.primitives[0].dimensions[1] = objects[i][4];
    object.primitives[0].dimensions[2] = objects[i][5];

    // apply collision object offset
    // right now just translates and rotates about z
    if(apply_offset_to_collision_objects_)
    {
      if(!collision_object_offset_.empty()) 
      {
        geometry_msgs::Pose p, p2;
        Eigen::Affine3d a;
        a(0,0) = cos(collision_object_offset_[5]);
        a(1,0) = sin(collision_object_offset_[5]);
        a(2,0) = 0;
        a(0,1) = -sin(collision_object_offset_[5]);
        a(1,1) = cos(collision_object_offset_[5]);
        a(2,1) = 0;
        a(0,2) = 0;
        a(1,2) = 0;
        a(2,2) = 0;
        leatherman::msgFromPose(a, p2);
        p2.position.x = 0;
        p2.position.y = 0;
        p2.position.z = 0;
        leatherman::multiplyPoses(p2, object.primitive_poses[0], p);
        p.position.x += collision_object_offset_pose_.position.x; 
        p.position.y += collision_object_offset_pose_.position.y; 
        p.position.z += collision_object_offset_pose_.position.z;
        object.primitive_poses[0] = p;
      }
      else
        ROS_ERROR("[exp] Expecting to translate/rotate collision objects in robot frame but offset not found.");
    }

    collision_objects.push_back(object);
    ROS_DEBUG("[exp] [%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f colors: %2.0f %2.0f %2.0f %2.0f",int(i),object_ids[i].c_str(),objects[i][0],objects[i][1],objects[i][2],objects[i][3],objects[i][4],objects[i][5], object_colors[i][0], object_colors[i][1], object_colors[i][2], object_colors[i][3]);
    ROS_INFO("[exp] [%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f colors: %2.0f %2.0f %2.0f %2.0f",int(i),object_ids[i].c_str(),object.primitive_poses[0].position.x,object.primitive_poses[0].position.y,object.primitive_poses[0].position.z, object.primitives[0].dimensions[0], object.primitives[0].dimensions[1], object.primitives[0].dimensions[2], object_colors[i][0], object_colors[i][1], object_colors[i][2], object_colors[i][3]);

    std::vector<double> dim(3,0);
    dim[0] = objects[i][3];
    dim[1] = objects[i][4];
    dim[2] = objects[i][5];
    pviz_.getCubeMsg(object.primitive_poses[0], dim, object_colors[i], "collision_objects", int(i), marker);
    marker_array.markers.push_back(marker);
  }
  ROS_INFO("[exp] I gathered %d collision cubes", int(marker_array.markers.size()));
  pviz_.publishMarkerArray(marker_array);
  usleep(500);
  return true;
}

bool BenchmarkManipulationTests::getAttachedObject(std::string object_file, geometry_msgs::Pose rarm_object_pose, moveit_msgs::AttachedCollisionObject &att_object)
{
  char sTemp[1024];
  float temp[6];
  tf::Quaternion q;
  att_object.link_name = "r_wrist_roll_link";
  att_object.touch_links.push_back("r_gripper");
  att_object.touch_links.push_back("l_gripper");
  att_object.touch_links.push_back("r_gripper_palm_link");
  att_object.touch_links.push_back("r_gripper_r_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("r_gripper_r_finger_tip_link");
  att_object.touch_links.push_back("r_wrist_roll_link");
  att_object.touch_links.push_back("l_gripper_palm_link");
  att_object.touch_links.push_back("l_gripper_r_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("l_gripper_r_finger_tip_link");
  att_object.touch_links.push_back("l_wrist_roll_link");
  att_object.object.header.frame_id = "r_wrist_roll_link";
  att_object.object.operation = moveit_msgs::CollisionObject::ADD;
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.primitives.resize(1);
  att_object.object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  att_object.object.primitive_poses.resize(1);

  FILE* fid = fopen(object_file.c_str(), "r");
  if(fid == NULL)
  {
    ROS_ERROR("[exp] Failed to open object file. (%s)", object_file.c_str());
    return false;
  }

  // object name
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  att_object.object.id = sTemp;
  // xyz in r_wrist_roll_link
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "xyz:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse xyz.");
    att_object.object.primitive_poses[0].position.x = temp[0];
    att_object.object.primitive_poses[0].position.y = temp[1];
    att_object.object.primitive_poses[0].position.z = temp[2];
    ROS_DEBUG("xyz: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  // rpy in r_wrist_roll_link
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "rpy:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse xyz.");
    q.setRPY(temp[0],temp[1],temp[2]);
    tf::quaternionTFToMsg(q, att_object.object.primitive_poses[0].orientation);
    ROS_DEBUG("rpy: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  // dims
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "dims:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse dims.");
    att_object.object.primitives[0].dimensions.resize(3,0);
    att_object.object.primitives[0].dimensions[0] = temp[0];
    att_object.object.primitives[0].dimensions[1] = temp[1];
    att_object.object.primitives[0].dimensions[2] = temp[2];  
    ROS_DEBUG("dims: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  return true;
}

bool BenchmarkManipulationTests::getAttachedObject(std::string object_file, geometry_msgs::Pose rarm_object_pose, arm_navigation_msgs::AttachedCollisionObject &att_object)
{
  char sTemp[1024];
  float temp[6];
  tf::Quaternion q;
  att_object.link_name = "r_wrist_roll_link";
  att_object.touch_links.push_back("r_gripper");
  att_object.touch_links.push_back("l_gripper");
  att_object.touch_links.push_back("r_gripper_palm_link");
  att_object.touch_links.push_back("r_gripper_r_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("r_gripper_r_finger_tip_link");
  att_object.touch_links.push_back("r_wrist_roll_link");
  att_object.touch_links.push_back("l_gripper_palm_link");
  att_object.touch_links.push_back("l_gripper_r_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("l_gripper_r_finger_tip_link");
  att_object.touch_links.push_back("l_wrist_roll_link");
  att_object.object.header.frame_id = "r_wrist_roll_link";
  att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.shapes.resize(1);
  att_object.object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
  att_object.object.poses.resize(1);

  FILE* fid = fopen(object_file.c_str(), "r");
  if(fid == NULL)
  {
    ROS_ERROR("[exp] Failed to open object file. (%s)", object_file.c_str());
    return false;
  }

  // object name
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  att_object.object.id = sTemp;
  // xyz in r_wrist_roll_link
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "xyz:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse xyz.");
    att_object.object.poses[0].position.x = temp[0];
    att_object.object.poses[0].position.y = temp[1];
    att_object.object.poses[0].position.z = temp[2];
    ROS_DEBUG("xyz: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  // rpy in r_wrist_roll_link
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "rpy:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse xyz.");
    q.setRPY(temp[0],temp[1],temp[2]);
    tf::quaternionTFToMsg(q, att_object.object.poses[0].orientation);
    ROS_DEBUG("rpy: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  // dims
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "dims:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse dims.");
    att_object.object.shapes[0].dimensions.resize(3,0);
    att_object.object.shapes[0].dimensions[0] = temp[0];
    att_object.object.shapes[0].dimensions[1] = temp[1];
    att_object.object.shapes[0].dimensions[2] = temp[2];  
    ROS_DEBUG("dims: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  return true;
}
void BenchmarkManipulationTests::visualizeEnvironment()
{
  ROS_ERROR("visualizeEnvironment does not work right now...");
}

bool BenchmarkManipulationTests::requestPlan(RobotPose &start_state, std::string name)
{
  bool sbpl_success = true;
  arm_navigation_msgs::GetMotionPlan::Request req;
  arm_navigation_msgs::GetMotionPlan::Response res;
  arm_navigation_msgs::PlanningScenePtr scene(new arm_navigation_msgs::PlanningScene);

  scene->collision_map.header.frame_id = world_frame_;
  scene->robot_state.joint_state.header.frame_id = robot_model_root_frame_;
  scene->robot_state.joint_state.header.stamp = ros::Time::now();
  scene->robot_state.joint_state.name.resize(1);
  scene->robot_state.joint_state.name[0] = "torso_lift_joint";
  scene->robot_state.joint_state.position.resize(1);
  scene->robot_state.joint_state.position[0] = start_state.body.z;

  // fill in collision objects
  // if(!known_objects_filename_.empty())
  // {
  //   if(!getCollisionObjects(known_objects_filename_, scene->collision_objects))
  //   {
  //     ROS_ERROR("[exp] Failed to get the collision objects from the file.");
  //     return false;
  //   }
  // }

  // fill in attached object
  if(exp_map_[name].pre_action == action_map_["attach"])
  {
    scene->attached_collision_objects.resize(1);
    if(!getAttachedObject(attached_object_filename_, rarm_object_pose_, scene->attached_collision_objects[0]))
    {
      ROS_ERROR("[exp] Failed to add the attached object.");
      return false;
    }
  }
  else if(exp_map_[name].pre_action == action_map_["detach"])
  {
    scene->attached_collision_objects.resize(1);
    if(!getAttachedObject(attached_object_filename_, rarm_object_pose_, scene->attached_collision_objects[0]))
    {
      ROS_ERROR("[exp] Failed to remove the attached object.");
      return false;
    }
    scene->attached_collision_objects[0].object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  }
// visualizeRobotPose(start_state, "start", 0);
  // sleep(0.5);
  // return false;
  // fill in goal
  fillSingleArmPlanningRequest(start_state,name,req.motion_plan_request);

  // filling planning scene with start state of joints so the rviz plugin can display them
  for(size_t i = 0; i < req.motion_plan_request.start_state.joint_state.name.size(); ++i)
  {
    scene->robot_state.joint_state.name.push_back(req.motion_plan_request.start_state.joint_state.name[i]);
    scene->robot_state.joint_state.position.push_back(req.motion_plan_request.start_state.joint_state.position[i]);
  }
  /*
  ROS_INFO("[exp] Publishing the planning scene for visualization using the motion_planning_rviz_plugin.");
  pscene_pub_.publish(&scene);
  // */
  // visualizeRobotPose(start_state, "start", 0);
  // sleep(0.5);

  KDL::Frame f;
  f.p.x(-0.05); f.p.y(0.0); f.p.z(0.789675 + start_state.body.z);
  f.M = KDL::Rotation::Quaternion(0,0,0,1);
  rm_->setKinematicsToPlanningTransform(f, world_frame_);

  scene->robot_state.multi_dof_joint_state.frame_ids.resize(1);
  scene->robot_state.multi_dof_joint_state.child_frame_ids.resize(1);
  scene->robot_state.multi_dof_joint_state.poses.resize(1);
  scene->robot_state.multi_dof_joint_state.frame_ids[0] = "base_footprint";
  scene->robot_state.multi_dof_joint_state.child_frame_ids[0] = "torso_lift_link";
  scene->robot_state.multi_dof_joint_state.poses[0].position.x = -0.05;
  scene->robot_state.multi_dof_joint_state.poses[0].position.y = 0.0;
  scene->robot_state.multi_dof_joint_state.poses[0].position.z = 0.789675 + start_state.body.z;
  scene->robot_state.multi_dof_joint_state.poses[0].orientation.w = 1;


  pviz_.deleteVisualizations("sbpl_planned_path", 3000);
  pviz_.deleteVisualizations("ompl_planned_path", 3000);
  pviz_.deleteVisualizations("ompl_simplified_path", 3000);
  pviz_.deleteVisualizations("sbpl_planned_execution", 100);
  pviz_.deleteVisualizations("ompl_planned_execution", 100);
  pviz_.deleteVisualizations("ompl_simplified_execution", 100);

  // sbpl plan
  ROS_INFO("[exp] Calling SBPL planner...");

  FILE* fin = fopen("/home/fahad/catkin_ws/src/single_arm/single_arm_benchmarks/benchmark_manipulation_tests/fbp_tests.yaml","r");
  if(!fin){
    printf("file %s does not exist\n");
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  bool first = true;
  int test_num = 0;
  std::map<std::string,Experiment>::iterator iter = exp_map_.begin();
  while(1)
  {
    
    name = iter->first;
    iter; //todoooo
    printf("nameeeeee %s\n", name.c_str());

    if(fscanf(fin,"  - test: test_%d\n    start:\n", &test_num) <= 0)
      break;

    if(fscanf(fin,"      rarm: %lf %lf %lf %lf %lf %lf %lf\n",
              &req2.rarm_start[0],&req2.rarm_start[1],
              &req2.rarm_start[2],&req2.rarm_start[3],
              &req2.rarm_start[4],&req2.rarm_start[5],
              &req2.rarm_start[6]) <= 0)
      break;
    fscanf(fin,"    goal:\n");
    if(fscanf(fin,"      rarm: %lf %lf %lf %lf %lf %lf %lf\n",
              &req2.rarm_goal[0],&req2.rarm_goal[1],
              &req2.rarm_goal[2],&req2.rarm_goal[3],
              &req2.rarm_goal[4],&req2.rarm_goal[5],
              &req2.rarm_goal[6]) <= 0)
      break;
      

    if(!sbpl_planner_->solve(scene, req, res, req2.rarm_start, req2.rarm_goal))
    {
      ROS_ERROR("[exp] SBPL failed to plan.");
      sbpl_success = false;
    }
    else
    {
      visualizeTrajectory(res.trajectory.joint_trajectory, start_state, 5, "sbpl_planned_path");
      visualizeTrajectoryExecution(res.trajectory.joint_trajectory, start_state, "sbpl_planned_execution", 0.05);
    }
    // ma_pub_.publish(cc_->getVisualization("bounds"));
    // ma_pub_.publish(cc_->getVisualization("distance_field"));
    // ma_pub_.publish(sbpl_planner_->getVisualization("goal"));
    // ma_pub_.publish(sbpl_planner_->getVisualization("expansions"));
    //ma_pub_.publish(cc_->getVisualization("collision_objects"));
    
    /* Note: Right now, it uses the final waypoint in the SBPL path as the OMPL goal. I did this really quickly as a hack to get it running, but a robust call to IK should be placed here so that the OMPL planner can be called regardless. */

    // ompl plan
    std::vector<double> ompl_times(2,0);
    std::vector<trajectory_msgs::JointTrajectoryPoint> ompl_ptraj, ompl_straj;
    if(sbpl_success)
    { 
      printf("OMPLLLLLLLLLLLLLL\n");
      // set start and goal
      // std::vector<double> start(7,0), goal(7,0);
      // for(size_t i = 0; i < res.trajectory.joint_trajectory.points[0].positions.size(); ++i)
      // {
      //   start[i] = res.trajectory.joint_trajectory.points[0].positions[i]; 
      //   goal[i] = res.trajectory.joint_trajectory.points.back().positions[i]; 
      // }
      ompl_planner_->setStartAndGoal(req2.rarm_start, req2.rarm_goal);

      if(!ompl_planner_->plan(req.motion_plan_request.allowed_planning_time.toSec(), ompl_ptraj, ompl_straj, ompl_times))
      {
        ROS_ERROR("[exp] OMPL failed to plan.");
      }
      else
      {
        
        pviz_.deleteVisualizations("sbpl_planned_execution", 100);
        trajectory_msgs::JointTrajectory otraj;
        otraj.points = ompl_ptraj;
        // visualizeTrajectory(otraj, start_state, 5, "ompl_planned_path");
        // visualizeTrajectoryExecution(otraj, start_state, "ompl_planned_execution", 0.1);
        pviz_.deleteVisualizations("ompl_planned_execution", 100);
        otraj.points = ompl_straj;
        // visualizeTrajectory(otraj, start_state, 5, "ompl_simplified_path");
        printf("showing ompl stats\n");
        // getchar();
        // visualizeTrajectoryExecution(otraj, start_state, "ompl_simplified_execution", 0.1);
      }
    }
    printf("test_num %d\n\n\n", test_num);
    // getchar();
    test_num++;
    // TODO: Make data structure to store planned paths, stats, descriptions etc.
    std::vector<trajectory_msgs::JointTrajectory> trajs(3);
    std::vector<double> times(3);
    std::vector<std::string> planners(3);
    std::vector<std::string> planner_ids(3);
    std::vector<std::string> descriptions(3);
    std::map<std::string, std::vector<double> > stats;
    trajs[0] = res.trajectory.joint_trajectory;
    trajs[1].points = ompl_ptraj;
    trajs[2].points = ompl_straj;
    planners[0] = "sbpl"; planners[1] = "ompl"; planners[2] = "ompl";
    planner_ids[0] = "jointspace"; planner_ids[1] = ompl_planner_id_; planner_ids[2] = ompl_planner_id_;
    descriptions[0] = "plan"; descriptions[1] = "plan"; descriptions[2] = "simplified";
    times[0] = res.planning_time.toSec(); times[1] = ompl_times[0]; times[2] = ompl_times[1];
    stats["processing_times"] = times;
    
    // update current robot pose
    ROS_INFO("[exp] Setting current pose using the %s path", planners[PLANNER_TO_SET_CURRENT_POSE].c_str());
    if(!trajs[PLANNER_TO_SET_CURRENT_POSE].points.empty())
    {
      if(!setRobotPoseFromTrajectory(trajs[PLANNER_TO_SET_CURRENT_POSE], req.motion_plan_request.start_state, current_pose_))
      {
        ROS_ERROR("[exp] Failed to set the current robot pose from the trajectory found.");
        return false;
      }
    }

    // write stats to file
    ROS_INFO("[exp] Recording stats to /tmp/benchmark_stats.csv");
    if(!writeStatsToFile(trajs, planners, planner_ids, descriptions, stats, name))
    {
      ROS_ERROR("[exp] Failed to write stats to file.");
      return false;
    } 

    // write trajectories to file
    ROS_INFO("[exp] Recording trajectories to file...");
    if(!writeTrajectoriesToFile(trajs, planners, planner_ids, descriptions, name))
    {
      ROS_ERROR("[exp] Failed to write the trajectories to file.");
      return false;
    }

    ROS_INFO("[exp] Planning request was a success.");
  }

  return true;
}

bool BenchmarkManipulationTests::runExperiment(std::string name)
{
  bool is_first_exp_ = false;
  RobotPose start;
  std::vector<std::vector<double> > traj;
 
  printf("************** %s **************\n", name.c_str()); 
 
  geometry_msgs::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 1.5;
  std::vector<double> color(4,1);
  // pviz_.visualizeText(pose, name, "experiment_name", 0, color, 0.1);

  std::map<std::string, Experiment>::iterator name_iter = exp_map_.find(name);
  if(std::distance(exp_map_.begin(), name_iter) == 0)
    is_first_exp_ = true;

  if(!use_current_state_as_start_ && is_first_exp_)
  {
    if(exp_map_[name].start.rangles.empty())
      start = start_pose_;
    else
      start = exp_map_[name].start;
  }
  else if(use_current_state_as_start_)
  {
    if(is_first_exp_)
      start = start_pose_;
    else
      start = current_pose_;
  }
  else if(use_initial_start_as_every_start_)
  {
    start = start_pose_;
  }
  else
    start = exp_map_[name].start;
  
  current_pose_ = start;

  if(use_current_state_as_start_)
    writeCompleteExperimentFile(exp_map_[name],start);

  // visualizeRobotPose(start, "start", 0);
  sleep(0.5);
  // printRobotPose(start, "start");
  // ROS_INFO("[exp]  goal: %s", exp_map_[name].goal.c_str());

  if(!requestPlan(start, name))
  {
    ROS_ERROR("[exp] %s failed to plan.", name.c_str());
    return false; 
  }
  else
    ROS_INFO("[exp] It's a planning miracle!");

  // visualizeLocations();
  return true;
}

bool BenchmarkManipulationTests::performAllExperiments()
{
  pviz_.deleteVisualizations("sbpl_planned_path", 3000);
  pviz_.deleteVisualizations("ompl_planned_path", 3000);
  pviz_.deleteVisualizations("ompl_simplified_path", 3000);
  pviz_.deleteVisualizations("collision_space_bounds", 300);
  pviz_.deleteVisualizations("distance_field", 5);
  pviz_.deleteVisualizations("experiment_name", 5);
  pviz_.deleteVisualizations("expansions", 5000);
  pviz_.deleteVisualizations("collision_objects", 55);

  if(use_current_state_as_start_)
    startCompleteExperimentFile();

  if(!createExperimentGroupFolder(experiment_group_name_))
  {
    ROS_ERROR("[exp] Failed to create experiment group folder for %s", experiment_group_name_.c_str());
    return false;
  }

  // for(std::map<std::string,Experiment>::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  // {
  std::map<std::string,Experiment>::iterator iter = exp_map_.begin();
    if(!runExperiment(iter->first))
      return false;

    // getchar();
 
    // visualizeRobotPose(current_pose_, "start", 0);
  // }

  /*
  if(!writePathSimilarityResultsToFile())
    ROS_ERROR("Failed to write consistency stats to file.");

  double sbpl_wrist_var=0, sbpl_elbow_var=0, ompl_wrist_var=0, ompl_elbow_var=0;
  std::vector<std::vector<geometry_msgs::Point> > paths, paths2;
  if(!getLinkPaths("SBPL", "", "simplify", "r_wrist_roll_link", paths, true, 1))
    ROS_ERROR("Failed to visualize the SBPL paths for the r_wrist_roll_link.");
  else
  { 
    if(!computePathSimilarity(paths, sbpl_wrist_var, paths2))
      ROS_ERROR("Failed to compute path similarity for SBPL wrist paths.");

    ROS_ERROR("%d points", int(paths2.size()));
    for(size_t i = 0; i < paths2.size(); ++i)
      pviz_.visualizeSpheres(paths2[i], 1, "sbpl-wrist", int(i), 0.01);
  }
  paths.clear();
  if(!getLinkPaths("SBPL", "", "simplify", "r_elbow_flex_link", paths, true, 60))
    ROS_ERROR("Failed to visualize the SBPL paths for the r_elbow_flex_link.");
  else
  {
    if(!computePathSimilarity(paths, sbpl_elbow_var, paths2))
      ROS_ERROR("Failed to compute path similarity for SBPL elbow paths.");
    ROS_ERROR("%d points", int(paths2.size()));
    for(size_t i = 0; i < paths2.size(); ++i)
      pviz_.visualizeSpheres(paths2[i], 60, "sbpl-elbow", int(i), 0.01);
  }
  paths.clear();
  if(!getLinkPaths("OMPL", "", "simplify", "r_wrist_roll_link", paths, true, 120))
    ROS_ERROR("Failed to visualize the OMPL paths for the r_wrist_roll_link.");
  else
  {
    if(!computePathSimilarity(paths, ompl_wrist_var, paths2))
      ROS_ERROR("Failed to compute path similarity for OMPL wrist paths.");
    ROS_ERROR("%d points", int(paths2.size()));
    for(size_t i = 0; i < paths2.size(); ++i)
      pviz_.visualizeSpheres(paths2[i], 120, "ompl-wrist", int(i), 0.01);
  }
  paths.clear();
  if(!getLinkPaths("OMPL", "", "simplify", "r_elbow_flex_link", paths, true, 230))
    ROS_ERROR("Failed to visualize the OMPL paths for the r_elbow_flex_link.");
  else
  {
    if(!computePathSimilarity(paths, ompl_elbow_var, paths2))
      ROS_ERROR("Failed to compute path similarity for OMPL elbow paths.");
    ROS_ERROR("%d points", int(paths2.size()));
    for(size_t i = 0; i < paths2.size(); ++i)
      pviz_.visualizeSpheres(paths2[i], 230, "ompl-elbow", int(i), 0.01);
  }
  ROS_INFO("[Path Similarity]");
  ROS_INFO("   [wrist]   sbpl: %0.4f   ompl: %0.4f", sbpl_wrist_var, ompl_wrist_var);
  ROS_INFO("   [elbow]   sbpl: %0.4f   ompl: %0.4f", sbpl_elbow_var, ompl_elbow_var);
  */

  return true;
}

void BenchmarkManipulationTests::visualizeStartPose()
{
  // visualizeRobotPose(start_pose_, "start", 0);
}

void BenchmarkManipulationTests::visualizeRobotPose(RobotPose &pose, std::string name, int id)
{
  pviz_.visualizeRobot(pose.rangles, pose.langles, pose.body, 120, name, id, true);
}

void BenchmarkManipulationTests::visualizeLocations()
{
  std::vector<std::vector<double> > poses;
  poses.resize(std::distance(exp_map_.begin(),exp_map_.end()));
  for(std::map<std::string,Experiment >::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  {
    int i = std::distance(exp_map_.begin(), iter);
    poses[i].resize(6,0.0);
    poses[i][0] = loc_map_[iter->second.goal].at(0);
    poses[i][1] = loc_map_[iter->second.goal].at(1);
    poses[i][2] = loc_map_[iter->second.goal].at(2);
    poses[i][3] = loc_map_[iter->second.goal].at(3);
    poses[i][4] = loc_map_[iter->second.goal].at(4);
    poses[i][5] = loc_map_[iter->second.goal].at(5);
  }

  ROS_INFO("[exp] Visualizing %d locations.", int(poses.size()));
  // pviz_.visualizePoses(poses);
}

void BenchmarkManipulationTests::startCompleteExperimentFile()
{
  FILE* fout = fopen("/tmp/completeExperiment.yaml","w");
  /*
  fprintf(fout,"goal_tolerance:\n  xyz: 0.02 0.02 0.02\n rpy: 0.05 0.05 0.05\n\n");
  fprintf(fout,"object_pose_in_gripper:\n  right:\n    xyz: -0.20 -0.1 0.0\n    rpy: 0.0 0.0 0.0\n  left:\n    xyz: -0.20 0.1 0.0\n    rpy: 0.0 0.0 0.0\n\n");
  fprintf(fout,"use_current_pose_as_start_state: false\n");
  fprintf(fout,"start_state:\n  right: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  left: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  base: 1.9 0.6 1.57\n  spine: 0.0\n\n");
  */
  fprintf(fout,"experiments:\n\n");
  fclose(fout);
}

void BenchmarkManipulationTests::writeCompleteExperimentFile(Experiment e, RobotPose start)
{
  std::string name = "/tmp/experiments_complete_" + experiment_group_name_ + ".yaml";
  FILE* fout = fopen(name.c_str(),"a");
  fprintf(fout,"  - name: %s\n",e.name.c_str());
  fprintf(fout,"    goal: %s\n",e.goal.c_str());
  if(e.pre_action==0)
    fprintf(fout,"    pre_action: nothing\n");
  else if(e.pre_action==1)
    fprintf(fout,"    pre_action: attach\n");
  else if(e.pre_action==2)
    fprintf(fout,"    pre_action: detach\n");
  if(e.post_action==0)
    fprintf(fout,"    post_action: nothing\n");
  else if(e.post_action==1)
    fprintf(fout,"    post_action: attach\n");
  else if(e.post_action==2)
    fprintf(fout,"    post_action: detach\n");
  fprintf(fout,"    sound_bite: \"%s\"\n",e.sound_bite.c_str());
  fprintf(fout,"    start:\n");
  fprintf(fout,"      right: %f %f %f %f %f %f %f\n",start.rangles[0],start.rangles[1],start.rangles[2],start.rangles[3],start.rangles[4],start.rangles[5],start.rangles[6]);
  fprintf(fout,"      left: %f %f %f %f %f %f %f\n",start.langles[0],start.langles[1],start.langles[2],start.langles[3],start.langles[4],start.langles[5],start.langles[6]);
  fprintf(fout,"      base: %f %f %f\n",start.body.x,start.body.y,start.body.theta);
  fprintf(fout,"      spine: %f\n",start.body.z);
  fclose(fout);
  
}

void BenchmarkManipulationTests::printLocations()
{
  if(loc_map_.begin() == loc_map_.end())
  {
    ROS_ERROR("[exp] No locations found.");
    return;
  }
  for(std::map<std::string,std::vector<double> >::const_iterator iter = loc_map_.begin(); iter != loc_map_.end(); ++iter)
  {
    ROS_INFO("name: %s", iter->first.c_str());
    ROS_INFO("  x: % 0.3f  y: % 0.3f  z: % 0.3f  roll: %0.3f  pitch: %0.3f  yaw: %0.3f", iter->second.at(0), iter->second.at(1), iter->second.at(2), iter->second.at(3), iter->second.at(4), iter->second.at(5));
  }
}

void BenchmarkManipulationTests::printExperiments()
{
  if(exp_map_.begin() == exp_map_.end())
  {
    ROS_ERROR("[exp] No experiments found.");
    return;
  }
  for(std::map<std::string,Experiment>::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  {
    int p = std::distance(exp_map_.begin(), iter);
    ROS_INFO("------------------------------");
    ROS_INFO("[%d] name: %s", p, iter->second.name.c_str());
    ROS_INFO("[%d] goal: %s", p, iter->second.goal.c_str());
    ROS_INFO("[%d] pre_action: %s", p, action_list_[iter->second.pre_action].c_str());
    ROS_INFO("[%d] post_action: %s", p, action_list_[iter->second.post_action].c_str());
    ROS_INFO("[%d] sound_bite: %s", p, iter->second.sound_bite.c_str());
    if(!use_current_state_as_start_ && !use_initial_start_as_every_start_)
    {
      ROS_INFO("[%d] start:", p);
      ROS_INFO("[%d]   right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", p, iter->second.start.rangles[0], iter->second.start.rangles[1], iter->second.start.rangles[2], iter->second.start.rangles[3], iter->second.start.rangles[4], iter->second.start.rangles[5], iter->second.start.rangles[6]);
      ROS_INFO("[%d]    left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", p, iter->second.start.langles[0], iter->second.start.langles[1], iter->second.start.langles[2], iter->second.start.langles[3], iter->second.start.langles[4], iter->second.start.langles[5], iter->second.start.langles[6]);
      ROS_INFO("[%d]    base: % 0.3f % 0.3f % 0.3f", p, iter->second.start.body.x, iter->second.start.body.y, iter->second.start.body.theta);
      ROS_INFO("[%d]   torso: % 0.3f", p, iter->second.start.body.z);
    }
  }
}

void BenchmarkManipulationTests::printRobotPose(RobotPose &pose, std::string name)
{
  if(pose.rangles.size() < 7 || pose.langles.size() < 7)
  {
    ROS_ERROR("[exp] Trying to print RobotPose but size of rangles = %d and size of langles = %d.", int(pose.rangles.size()), int(pose.langles.size()));
    return;
  }
  ROS_INFO("[%s] right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), pose.rangles[0], pose.rangles[1], pose.rangles[2], pose.rangles[3], pose.rangles[4], pose.rangles[5], pose.rangles[6]);
  ROS_INFO("[%s]  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), pose.langles[0], pose.langles[1], pose.langles[2], pose.langles[3], pose.langles[4], pose.langles[5], pose.langles[6]);
  ROS_INFO("[%s]  base: % 0.3f % 0.3f % 0.3f", name.c_str(), pose.body.x, pose.body.y, pose.body.theta);
  ROS_INFO("[%s] torso: % 0.3f", name.c_str(), pose.body.z);
}

void BenchmarkManipulationTests::printParams()
{
  for(size_t i = 0; i < planner_interfaces_.size(); ++i)
    ROS_INFO("[planner_interface] %d/%d %s", int(i+1), int(planner_interfaces_.size()), planner_interfaces_[i].c_str());

  ROS_INFO("     [goal tolerance] x: % 0.3f  y: % 0.3f  z: % 0.3f  roll: % 0.3f  pitch: %0.3f  yaw: %0.3f", goal_tolerance_[0], goal_tolerance_[1], goal_tolerance_[2], goal_tolerance_[3], goal_tolerance_[4], goal_tolerance_[5]);
  ROS_INFO("  [right object pose] x: % 0.3f  y: % 0.3f  z: % 0.3f  r: % 0.3f  p: % 0.3f  y: % 0.3f", rarm_object_offset_[0], rarm_object_offset_[1], rarm_object_offset_[2], rarm_object_offset_[3], rarm_object_offset_[4], rarm_object_offset_[5]);
  ROS_INFO("   [left object pose] x: % 0.3f  y: % 0.3f  z: % 0.3f  r: % 0.3f  p: % 0.3f  y: % 0.3f", larm_object_offset_[0], larm_object_offset_[1], larm_object_offset_[2], larm_object_offset_[3], larm_object_offset_[4], larm_object_offset_[5]);
  ROS_INFO("[initial start state] right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_pose_.rangles[0], start_pose_.rangles[1], start_pose_.rangles[2], start_pose_.rangles[3], start_pose_.rangles[4], start_pose_.rangles[5], start_pose_.rangles[6]);
  ROS_INFO("[initial start state]  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_pose_.langles[0], start_pose_.langles[1], start_pose_.langles[2], start_pose_.langles[3], start_pose_.langles[4], start_pose_.langles[5], start_pose_.langles[6]);
  ROS_INFO("[initial start state]  base: % 0.3f % 0.3f % 0.3f", start_pose_.body.x, start_pose_.body.y, start_pose_.body.theta);
  ROS_INFO("[initial start state] torso: % 0.3f", start_pose_.body.z);

  printLocations();
  printExperiments();
}

void BenchmarkManipulationTests::fillSingleArmPlanningRequest(RobotPose &start_state, std::string name, moveit_msgs::MotionPlanRequest &req)
{
  tf::Quaternion q;
  req.group_name = group_name_;
  req.num_planning_attempts = 1;
  req.allowed_planning_time = 10.0;
  req.planner_id = ompl_planner_id_;

  if(start_state.rangles.size() != rjoint_names_.size())
    ROS_ERROR("[exp] Didn't receive expected number of right arm joint angles in start state.");
  if(start_state.langles.size() != ljoint_names_.size())
    ROS_ERROR("[exp] Didn't receive expected number of left arm joint angles in start state.");

  // start state
  for(size_t i = 0; i < start_state.rangles.size(); ++i)
  {
    req.start_state.joint_state.position.push_back(start_state.rangles[i]);
    req.start_state.joint_state.name.push_back(rjoint_names_[i]);
  }
  for(size_t i = 0; i < start_state.langles.size(); ++i)
  {
    req.start_state.joint_state.position.push_back(start_state.langles[i]);
    req.start_state.joint_state.name.push_back(ljoint_names_[i]);
  }

  req.start_state.joint_state.position.push_back(start_state.body.z);
  req.start_state.joint_state.name.push_back("torso_lift_joint");

  // goal pose
  req.goal_constraints.resize(1);
  req.goal_constraints[0].position_constraints.resize(1);
  req.goal_constraints[0].position_constraints[0].header.stamp = ros::Time::now();
  req.goal_constraints[0].position_constraints[0].header.frame_id = world_frame_;

  req.goal_constraints[0].position_constraints[0].link_name = "r_wrist_roll_link";
  req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.resize(1);
  req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.x = loc_map_[exp_map_[name].goal].at(0);
  req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.y = loc_map_[exp_map_[name].goal].at(1);
  req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.z = loc_map_[exp_map_[name].goal].at(2);
  req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].orientation.w = 1;

  req.goal_constraints[0].position_constraints[0].constraint_region.primitives.resize(1);
  req.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  req.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.push_back(goal_tolerance_[0]);
  req.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.push_back(goal_tolerance_[1]);
  req.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.push_back(goal_tolerance_[2]);
  req.goal_constraints[0].position_constraints[0].weight = 1.0;

  req.goal_constraints[0].orientation_constraints.resize(1);
  req.goal_constraints[0].orientation_constraints[0].header.stamp = ros::Time::now();
  req.goal_constraints[0].orientation_constraints[0].header.frame_id = world_frame_;
  req.goal_constraints[0].orientation_constraints[0].link_name = "r_wrist_roll_link";

  q.setRPY(loc_map_[exp_map_[name].goal].at(3), loc_map_[exp_map_[name].goal].at(4), loc_map_[exp_map_[name].goal].at(5));
  tf::quaternionTFToMsg(q, req.goal_constraints[0].orientation_constraints[0].orientation);

  req.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance = goal_tolerance_[3];
  req.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance = goal_tolerance_[4];
  req.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance = goal_tolerance_[5];
  req.goal_constraints[0].orientation_constraints[0].weight = 1.0;

  ROS_INFO("[exp] [goal] rpy: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f", loc_map_[exp_map_[name].goal].at(3), loc_map_[exp_map_[name].goal].at(4), loc_map_[exp_map_[name].goal].at(5), req.goal_constraints[0].orientation_constraints[0].orientation.x, req.goal_constraints[0].orientation_constraints[0].orientation.y, req.goal_constraints[0].orientation_constraints[0].orientation.z, req.goal_constraints[0].orientation_constraints[0].orientation.w);
}

void BenchmarkManipulationTests::fillSingleArmPlanningRequest(RobotPose &start_state, std::string name, arm_navigation_msgs::MotionPlanRequest &req)
{
  tf::Quaternion q;
  req.group_name = group_name_;
  req.num_planning_attempts = 1;
  req.allowed_planning_time.fromSec(30.0);
  req.planner_id = ompl_planner_id_;

  if(start_state.rangles.size() != rjoint_names_.size())
    ROS_ERROR("[exp] Didn't receive expected number of right arm joint angles in start state.");
  if(start_state.langles.size() != ljoint_names_.size())
    ROS_ERROR("[exp] Didn't receive expected number of left arm joint angles in start state.");

  // start state
  for(size_t i = 0; i < start_state.rangles.size(); ++i)
  {
    req.start_state.joint_state.position.push_back(start_state.rangles[i]);
    req.start_state.joint_state.name.push_back(rjoint_names_[i]);
  }
  for(size_t i = 0; i < start_state.langles.size(); ++i)
  {
    req.start_state.joint_state.position.push_back(start_state.langles[i]);
    req.start_state.joint_state.name.push_back(ljoint_names_[i]);
  }

  req.start_state.joint_state.position.push_back(start_state.body.z);
  req.start_state.joint_state.name.push_back("torso_lift_joint");

  // goal pose
  req.goal_constraints.position_constraints.resize(1);
  req.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  req.goal_constraints.position_constraints[0].header.frame_id = world_frame_;

  req.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  req.goal_constraints.position_constraints[0].position.x = loc_map_[exp_map_[name].goal].at(0);
  req.goal_constraints.position_constraints[0].position.y = loc_map_[exp_map_[name].goal].at(1);
  req.goal_constraints.position_constraints[0].position.z = loc_map_[exp_map_[name].goal].at(2);

  req.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.resize(3);
  req.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  req.goal_constraints.position_constraints[0].constraint_region_shape.dimensions[0] = goal_tolerance_[0];
  req.goal_constraints.position_constraints[0].constraint_region_shape.dimensions[1] = goal_tolerance_[1];
  req.goal_constraints.position_constraints[0].constraint_region_shape.dimensions[2] = goal_tolerance_[2];
  req.goal_constraints.position_constraints[0].constraint_region_orientation.w = 0;
  req.goal_constraints.position_constraints[0].weight = 1.0;

  req.goal_constraints.orientation_constraints.resize(1);
  req.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  req.goal_constraints.orientation_constraints[0].header.frame_id = world_frame_;
  req.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";

  q.setRPY(loc_map_[exp_map_[name].goal].at(3), loc_map_[exp_map_[name].goal].at(4), loc_map_[exp_map_[name].goal].at(5));
  tf::quaternionTFToMsg(q, req.goal_constraints.orientation_constraints[0].orientation);

  req.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = goal_tolerance_[3];
  req.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = goal_tolerance_[4];
  req.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = goal_tolerance_[5];
  req.goal_constraints.orientation_constraints[0].weight = 1.0;

  ROS_INFO("[exp] [goal] rpy: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f", loc_map_[exp_map_[name].goal].at(3), loc_map_[exp_map_[name].goal].at(4), loc_map_[exp_map_[name].goal].at(5), req.goal_constraints.orientation_constraints[0].orientation.x, req.goal_constraints.orientation_constraints[0].orientation.y, req.goal_constraints.orientation_constraints[0].orientation.z, req.goal_constraints.orientation_constraints[0].orientation.w);
}

bool BenchmarkManipulationTests::setRobotPoseFromTrajectory(moveit_msgs::RobotTrajectory &trajectory, moveit_msgs::RobotState &trajectory_start, RobotPose &pose)
{
  // update the current pose with the previous start state of the robot
  getRobotPoseFromRobotState(trajectory_start, pose);

  if(trajectory.joint_trajectory.points.empty())
  {
    ROS_INFO("[exp] Trajectory is empty....unable to set robot pose.");
    return true;
  }

  // gets right arm joints only
  if(!getJointPositionsFromTrajectory(trajectory.joint_trajectory, rjoint_names_, int(trajectory.joint_trajectory.points.size())-1, pose.rangles))
  {
    ROS_ERROR("[exp] Failed to get the joint positions for the right arm from waypoint %d", int(trajectory.joint_trajectory.points.size())-1);
    return false;
  }
  return true;
}

bool BenchmarkManipulationTests::setRobotPoseFromTrajectory(trajectory_msgs::JointTrajectory &trajectory, arm_navigation_msgs::RobotState &trajectory_start, RobotPose &pose)
{
  // update the current pose with the previous start state of the robot
  getRobotPoseFromRobotState(trajectory_start, pose);

  if(trajectory.points.empty())
  {
    ROS_INFO("[exp] Trajectory is empty....unable to set robot pose.");
    return true;
  }

  // gets right arm joints only
  if(!getJointPositionsFromTrajectory(trajectory, rjoint_names_, int(trajectory.points.size())-1, pose.rangles))
  {
    ROS_ERROR("[exp] Failed to get the joint positions for the right arm from waypoint %d", int(trajectory.points.size())-1);
    return false;
  }
  return true;
}

bool BenchmarkManipulationTests::getJointPositionsFromTrajectory(const trajectory_msgs::JointTrajectory &traj, std::vector<std::string> &names, int waypoint, std::vector<double> &angles)
{
  unsigned int ind = 0;
  angles.resize(names.size());

  for(size_t i = 0; i < traj.joint_names.size(); i++)
  {
    if(names[ind].compare(traj.joint_names[i]) == 0)
    {
      angles[ind] = traj.points[waypoint].positions[i];
      ind++;
      ROS_DEBUG("[exp] Found %s in trajectory, ind = %d", traj.joint_names[i].c_str(), ind);
    }
    if(ind == names.size())
      break;
  }
  if(ind != names.size())
  {
    ROS_WARN("[exp] Not all of the expected joints were found in the trajectory.");
    return false;
  }
  return true;
}

bool BenchmarkManipulationTests::getRobotPoseFromRobotState(const moveit_msgs::RobotState &state, RobotPose &pose)
{
  unsigned int lind = 0, rind = 0;
  pose.langles.resize(ljoint_names_.size());
  pose.rangles.resize(rjoint_names_.size());

  // arms
  for(size_t i = 0; i < state.joint_state.name.size(); i++)
  {
    if(rind < rjoint_names_.size())
    {
      if(rjoint_names_[rind].compare(state.joint_state.name[i]) == 0)
      {
        ROS_DEBUG("[exp] [right-start] %-20s: %0.3f", rjoint_names_[rind].c_str(), state.joint_state.position[i]);
        pose.rangles[rind] = state.joint_state.position[i];
        rind++;
      }
    }
    if(lind < ljoint_names_.size())
    {
      if(ljoint_names_[lind].compare(state.joint_state.name[i]) == 0)
      {
        ROS_DEBUG("[exp] [left-start] %-20s: %0.3f", ljoint_names_[lind].c_str(), state.joint_state.position[i]);
        pose.langles[lind] = state.joint_state.position[i];
        lind++;
      }
    }
    if(rind == rjoint_names_.size() && lind == ljoint_names_.size())
      break;
  }
  if(rind != rjoint_names_.size() || lind != ljoint_names_.size())
  {
    ROS_DEBUG("[exp] Not all of the expected joints were found in the RobotState.");
    return false;
  }

  // torso
  for(size_t i = 0; i < state.joint_state.name.size(); i++)
  {
    if(state.joint_state.name[i].compare(spine_frame_) == 0)
    {
      pose.body.z = state.joint_state.position[i];
      break;
    }
  }

  return true;
}

bool BenchmarkManipulationTests::getRobotPoseFromRobotState(const arm_navigation_msgs::RobotState &state, RobotPose &pose)
{
  unsigned int lind = 0, rind = 0;
  pose.langles.resize(ljoint_names_.size());
  pose.rangles.resize(rjoint_names_.size());

  // arms
  for(size_t i = 0; i < state.joint_state.name.size(); i++)
  {
    if(rind < rjoint_names_.size())
    {
      if(rjoint_names_[rind].compare(state.joint_state.name[i]) == 0)
      {
        ROS_DEBUG("[exp] [right-start] %-20s: %0.3f", rjoint_names_[rind].c_str(), state.joint_state.position[i]);
        pose.rangles[rind] = state.joint_state.position[i];
        rind++;
      }
    }
    if(lind < ljoint_names_.size())
    {
      if(ljoint_names_[lind].compare(state.joint_state.name[i]) == 0)
      {
        ROS_DEBUG("[exp] [left-start] %-20s: %0.3f", ljoint_names_[lind].c_str(), state.joint_state.position[i]);
        pose.langles[lind] = state.joint_state.position[i];
        lind++;
      }
    }
    if(rind == rjoint_names_.size() && lind == ljoint_names_.size())
      break;
  }
  if(rind != rjoint_names_.size() || lind != ljoint_names_.size())
  {
    ROS_DEBUG("[exp] Not all of the expected joints were found in the RobotState.");
    return false;
  }

  // torso
  for(size_t i = 0; i < state.joint_state.name.size(); i++)
  {
    if(state.joint_state.name[i].compare(spine_frame_) == 0)
    {
      pose.body.z = state.joint_state.position[i];
      break;
    }
  }

  return true;
}

bool BenchmarkManipulationTests::getBasePoseFromPlanningScene(const moveit_msgs::PlanningScene &scene, BodyPose &body)
{
  // base - if there is a map frame
  if(!scene.fixed_frame_transforms.empty())
  {
    for(size_t i = 0; i < scene.fixed_frame_transforms.size(); ++i)
    {
      if(scene.fixed_frame_transforms[i].header.frame_id.compare(world_frame_) == 0 && 
          scene.fixed_frame_transforms[i].child_frame_id.compare(robot_model_root_frame_) == 0)
      {
        body.x = scene.fixed_frame_transforms[i].transform.translation.x;
        body.y = scene.fixed_frame_transforms[i].transform.translation.y;
        body.theta =  2 * atan2(scene.fixed_frame_transforms[i].transform.rotation.z, scene.fixed_frame_transforms[i].transform.rotation.w);
        ROS_WARN("WARNING: The formula to compute the yaw of the base pose is untested.");
        break;
      }
    }
  }
  return true;
}

bool BenchmarkManipulationTests::printPathToFile(FILE** file, const trajectory_msgs::JointTrajectory &traj)
{
  if(*file == NULL)
  {
    ROS_ERROR("[exp] File pointer is null.");
    return false;
  }

  double roll,pitch,yaw,roll1,pitch1,yaw1;
  tf::Pose tf_pose;
  std::vector<geometry_msgs::Pose> poses(2);
  std::vector<double> jnt_pos(7,0);

  for(size_t i = 0; i < traj.points.size(); ++i)
  {
    for(size_t j = 0; j < 7; ++j)
      jnt_pos[j] = traj.points[i].positions[j];

    /*
    if(!computeFK(jnt_pos, poses))
    {
      ROS_ERROR("[node] IK failed when printing path to file.");
      return false;
    }
    // HACK: Assumes fk_links_ is just set for elbow and gripper
    if(poses.size() < 2)
    { 
      ROS_ERROR("Expected 2 poses out of computeFK function. Something is wrong. Not writing to file.");
      return false;
    }
    */
    tf::poseMsgToTF(poses[0], tf_pose);
    tf_pose.getBasis().getRPY(roll,pitch,yaw);
    tf::poseMsgToTF(poses[1], tf_pose);
    tf_pose.getBasis().getRPY(roll1,pitch1,yaw1);


    // HACK: the "," at the end is needed so that it can be parsed by PViz::parseCSVFile
    if(experiment_type_ == 1)
      fprintf(*file, "%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %2.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %2.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f,\n", traj.points[i].positions[0],traj.points[i].positions[1],traj.points[i].positions[2],traj.points[i].positions[3],traj.points[i].positions[4],traj.points[i].positions[5],traj.points[i].positions[6],poses[0].position.x, poses[0].position.y, poses[0].position.z, roll, pitch, yaw, poses[0].orientation.x,poses[0].orientation.y, poses[0].orientation.z, poses[0].orientation.w, poses[1].position.x, poses[1].position.y, poses[1].position.z, roll1, pitch1, yaw1, poses[1].orientation.x,poses[1].orientation.y, poses[1].orientation.z, poses[1].orientation.w);
    else
      ROS_ERROR("Print function only implemented for single arm planning. Remove the experiment_type param.");
  }
  fflush(*file);
  return true;
}

bool BenchmarkManipulationTests::computeFK(std::vector<double> &angles, std::vector<geometry_msgs::Pose> &poses)
{
  ROS_WARN("ComputeFK is not implemented.");
  return false;
}

bool BenchmarkManipulationTests::createTrajectoryFile(std::string exp_name, std::string planner, std::string planner_id, std::string description, FILE** file, std::string &filename)
{
  size_t slash = planner.find("/");
  planner = planner.substr(slash+1);
  if(planner_id.empty())
    filename  = exp_name + "-" + planner + "_" + description + ".csv";
  else
    filename  = exp_name + "-" + planner + "_" + planner_id + "_" + description + ".csv";

  filename = trajectory_files_path_ + "/" +  filename;
  if((*file = fopen(filename.c_str(), "w")) == NULL)
  {
    ROS_ERROR("[node] Failed to create trajectory file. (%s)", filename.c_str());
    return false;
  }
  ROS_INFO("[exp] Successfully created trajectory file: %s", filename.c_str());
  return true;
}

bool BenchmarkManipulationTests::createExperimentGroupFolder(std::string exp_group_name)
{
  std::string folder = benchmark_results_folder_ + exp_group_name;
  if(!leatherman::createFolder(folder))
    return false;
  
  folder = folder + "/" + "trajectories/";
  if(!leatherman::createFolder(folder))
    return false;
   
  return true;
}

bool BenchmarkManipulationTests::createTrajectoryFolder(std::string exp_name)
{
  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));
  time.erase(time.size()-1, 1);

  std::string folder = benchmark_results_folder_ + experiment_group_name_ + "/trajectories/" + exp_name; // + "_" + time;
  if(!leatherman::createFolder(folder))
  {
    ROS_WARN("[exp] Failed to create the trajectory folder: %s. Maybe it exists already?", folder.c_str());
    return false;
  }
  trajectory_files_path_ = folder;
  return true;
}

bool BenchmarkManipulationTests::writeTrajectoriesToFile(const std::vector<trajectory_msgs::JointTrajectory> &trajs, std::vector<std::string> &planners, std::vector<std::string> &planner_ids, std::vector<std::string> &descriptions, std::string exp_name)
{
  FILE* fptr = NULL;
  std::vector<std::vector<std::string> > filenames(trajs.size());
  if(!createTrajectoryFolder(exp_name))
  {
    ROS_ERROR("[exp] Failed to create a trajectory folder for %s", exp_name.c_str());
    return false;
  }

  for(size_t i = 0; i < trajs.size(); ++i)
  {
    filenames[i].resize(1);
    if(!createTrajectoryFile(exp_name, planners[i], planner_ids[i], descriptions[i], &fptr, filenames[i][0]))
    {
      ROS_ERROR("[exp] Failed to create a trajectory file for %s + %s + %s", planners[i].c_str(), planner_ids[i].c_str(), descriptions[i].c_str());
      return false;
    }

    if(!printPathToFile(&fptr, trajs[i]))
    {
      ROS_ERROR("[exp] Failed to print the trajectory to file.");
      return false;
    }
    fclose(fptr);
  }  
  trajectory_file_map_[exp_name] = filenames; 
  return true;
}

bool BenchmarkManipulationTests::getPathFromFile(std::string filename, trajectory_msgs::JointTrajectory &traj, std::vector<std::vector<geometry_msgs::Pose> > &poses)
{
  std::vector<std::vector<double> > path;
  if(!getPathFromFile(filename, path))
    return false;

  traj.points.resize(path.size());
  poses.resize(path.size());
  for(size_t i = 0; i < path.size(); ++i)
  {
    traj.points[i].positions.resize(num_joints_);
    for(size_t j = 0; j < traj.points[i].positions.size(); ++j)
      traj.points[i].positions[j] = path[i][j];

    // each link pose has 10 elements: {x,y,z,roll,pitch,yaw,quatx,quaty,quatz,quatw}
    double num_poses = (double(path[i].size()) - num_joints_) / 10.0;
    if(floor((double(path[i].size()) - num_joints_) / 10.0) - num_poses > 0.0)
      return false;

    poses[i].resize(num_poses);
    for(int j = 0; j < int(poses[i].size()); ++j)
    {
      poses[i][j].position.x = path[i][num_joints_ + (j*10) + 0];
      poses[i][j].position.y = path[i][num_joints_ + (j*10) + 1];
      poses[i][j].position.z = path[i][num_joints_ + (j*10) + 2];
      poses[i][j].orientation.x = path[i][num_joints_ + (j*10) + 6];
      poses[i][j].orientation.y = path[i][num_joints_ + (j*10) + 7];
      poses[i][j].orientation.z = path[i][num_joints_ + (j*10) + 8];
      poses[i][j].orientation.w = path[i][num_joints_ + (j*10) + 9];
    }      
  }

  ROS_DEBUG("Parsed Path with %d waypoints and %d link paths from %s", int(traj.points.size()), int(poses[0].size()), filename.c_str());
  for(size_t i = 0; i < poses.size(); ++i)
    for(size_t j = 0; j < poses[i].size(); ++j)
      ROS_DEBUG("[%d-%d] xyz: %2.3f %2.3f %2.3f   quaternion:  %2.3f %2.3f %2.3f %2.3f", int(i), int(j), poses[i][j].position.x, poses[i][j].position.y, poses[i][j].position.z, poses[i][j].orientation.x, poses[i][j].orientation.y, poses[i][j].orientation.z, poses[i][j].orientation.w);

  return true;
}

bool BenchmarkManipulationTests::getPathFromFile(std::string filename, std::vector<std::vector<double> > &path)
{
  if(!pviz_.parseCSVFile(filename, 27, path))
    return false;
  return true;
}

bool BenchmarkManipulationTests::writeStatsToFile(const std::vector<trajectory_msgs::JointTrajectory> &trajs, std::vector<std::string> &planners, std::vector<std::string> &planner_ids, std::vector<std::string> &descriptions, std::map<std::string, std::vector<double> > &stats, std::string exp_name)
{
  std::string status;
  FILE* fptr = NULL;
  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));
  time.erase(time.size()-1, 1);

  std::string filename = "/tmp/benchmark_stats.csv";
  if((fptr = fopen(filename.c_str(), "a")) == NULL)
  {
    ROS_ERROR("[exp] Failed to create stats file. (%s)", filename.c_str());
    return false;
  }
  ROS_INFO("[exp] Successfully created stats file: %s", filename.c_str());

  for(size_t i = 0; i < 2; ++i)
  {
    fprintf(fptr, "%s, %s, %s, ", exp_name.c_str(), planners[i].c_str(), planner_ids[i].c_str());
    
    if(trajs[i].points.empty())
      status = "FAIL";  
    else
      status = "success";

    fprintf(fptr, "%s, %s, ", descriptions[i].c_str(), status.c_str());

    for(std::map<std::string,std::vector<double> >::const_iterator iter = stats.begin(); iter != stats.end(); ++iter)
      fprintf(fptr, "%0.5f, ", iter->second[i]);

    fprintf(fptr, "%d, %s\n", int(trajs[i].points.size()), time.c_str());
  }
  fclose(fptr);

  ROS_INFO("[exp] Successfully wrote the stats to a file.");
  return true;
}

void BenchmarkManipulationTests::printTrajectoryFilenames()
{
  if(trajectory_file_map_.begin() == trajectory_file_map_.end())
  {
    ROS_ERROR("[exp] No trajectory filenames found.");
    return;
  }
  for(std::map<std::string,std::vector<std::vector<std::string> > >::const_iterator iter = trajectory_file_map_.begin(); iter != trajectory_file_map_.end(); ++iter)
  {
    ROS_INFO("experiment: %s", iter->first.c_str());
    for(size_t i = 0; i < iter->second.size(); ++i)
      for(size_t j = 0; j < iter->second[i].size(); ++j)
        ROS_INFO("  [planner: %d] [path: %d] %s", int(i), int(j), iter->second[i][j].c_str());
  }
}

bool BenchmarkManipulationTests::getLinkPaths(std::string planner_interface, std::string planner_id, std::string description, std::string link_name, std::vector<std::vector<geometry_msgs::Point> > &paths, bool visualize, int hue)
{
  trajectory_msgs::JointTrajectory traj;
  std::vector<std::vector<geometry_msgs::Pose> > poses;
  std::vector<geometry_msgs::Point> link_path;  
  if(trajectory_file_map_.begin() == trajectory_file_map_.end())
  {
    ROS_ERROR("[exp] No trajectory filenames found.");
    return false;
  }

  int link_index = 0;
  bool link_found = false;
  for(size_t i = 0; i < fk_link_.size(); ++i)
  {
    if(fk_link_[i].compare(link_name) == 0)
    {
      link_found = true;
      link_index = i;
      break;
    }
  }
  if(!link_found)
  {
    ROS_ERROR("Can't collect paths for link, '%s', because it wasn't found in the trajectory file.", link_name.c_str());
    return false;
  }

  paths.clear();
  for(std::map<std::string,std::vector<std::vector<std::string> > >::const_iterator iter = trajectory_file_map_.begin(); iter != trajectory_file_map_.end(); ++iter)
  {
    for(size_t i = 0; i < iter->second.size(); ++i)
    {
      for(size_t j = 0; j < iter->second[i].size(); ++j)
      {
        if(iter->second[i][j].find(planner_interface) == std::string::npos)
          continue;

        if(iter->second[i][j].find(description) == std::string::npos)
          continue;

        poses.clear();
        if(!getPathFromFile(iter->second[i][j], traj, poses))
        {
          ROS_ERROR("Failed to parse a trajectory file. (%s)", iter->second[i][j].c_str()); 
          continue;
        }
        if(poses.empty())
          continue;

        if(link_index > int(poses[0].size()) || link_index < 0)
          return false;

        // get path for the link we want, convert from pose to point 
        link_path.resize(poses.size());
        for(size_t k = 0; k < poses.size(); ++k)
          link_path[k] = poses[k][link_index].position;

        paths.push_back(link_path);
      }
    }
  }
  if(paths.empty())
    return false;

  if(visualize)
  {
    for(size_t i = 0; i < paths.size(); ++i)
      pviz_.visualizeLine(paths[i], link_name+"_paths-"+planner_interface+"_"+description, i, hue, 0.005);
  }

  return true;
}

bool BenchmarkManipulationTests::computePathSimilarity(std::string planner_interface, std::string planner_id, std::string description, std::string link_name, double &variance, bool use_dtw)
{
  trajectory_msgs::JointTrajectory traj;
  std::vector<std::vector<geometry_msgs::Pose> > poses;
  std::vector<geometry_msgs::Point> link_path;
  std::vector<std::vector<geometry_msgs::Point> > paths;
  if(trajectory_file_map_.begin() == trajectory_file_map_.end())
  {
    ROS_ERROR("[exp] No trajectory filenames found.");
    return false;
  }

  int link_index = 0;
  bool link_found = false;
  for(size_t i = 0; i < fk_link_.size(); ++i)
  {
    if(fk_link_[i].compare(link_name) == 0)
    {
      link_found = true;
      link_index = i;
      break;
    }
  }
  if(!link_found)
  {
    ROS_ERROR("Can't compute path similarity for link, '%s', because it wasn't found in the trajectory file.", link_name.c_str());
    return false;
  }

  for(std::map<std::string,std::vector<std::vector<std::string> > >::const_iterator iter = trajectory_file_map_.begin(); iter != trajectory_file_map_.end(); ++iter)
  {
    for(size_t i = 0; i < iter->second.size(); ++i)
    {
      for(size_t j = 0; j < iter->second[i].size(); ++j)
      {
        if(iter->second[i][j].find(planner_interface) == std::string::npos)
          continue;

        if(iter->second[i][j].find(description) == std::string::npos)
          continue;

        poses.clear();
        if(!getPathFromFile(iter->second[i][j], traj, poses))
        {
          ROS_ERROR("Failed to parse a trajectory file. (%s)", iter->second[i][j].c_str()); 
          continue;
        }
        if(poses.empty())
          continue;

        if(link_index > int(poses[0].size()) || link_index < 0)
          return false;

        // get path for the link we want, convert from pose to point 
        link_path.resize(poses.size());
        for(size_t k = 0; k < poses.size(); ++k)
          link_path[k] = poses[k][link_index].position;

        paths.push_back(link_path);
      }
    }
  }
  ROS_INFO("Found %d paths from %s of type, '%s' (%s)", int(paths.size()), planner_interface.c_str(), description.c_str(), link_name.c_str());
  if(paths.empty())
  {
    variance = -1;
    return false;
  }

  /*
  for(size_t i = 0; i < paths.size(); ++i)
    pviz_.visualizeLine(paths[i], link_name+"_paths-"+planner_interface+"_"+description, i,(i+1)*(255/int(paths.size())), 0.005);
  */
  if(!use_dtw)
  {
    if(!computePathSimilarity(paths, variance))
    {
      ROS_ERROR("Successfully got the paths but measure() failed.");
      return false;
    }
  }
  else
  {
    if(!computePathSimilarityDTW(paths, variance))
    {
      ROS_ERROR("Successfully got the paths but measureDTW() failed.");
      return false;
    }
  }

  return true;
}

bool BenchmarkManipulationTests::computePathSimilarity(const std::vector<std::vector<geometry_msgs::Point> > &paths, double &variance)
{
  std::vector<const sbpl::PathSimilarityMeasurer::Trajectory*> trajs(paths.size());
  for(size_t i = 0; i < paths.size(); ++i)
    trajs[i] = &(paths[i]);

  variance = sbpl::PathSimilarityMeasurer::measure(trajs, 100);
  return true;
}

bool BenchmarkManipulationTests::computePathSimilarityDTW(const std::vector<std::vector<geometry_msgs::Point> > &paths, double &variance)
{
  std::vector<const sbpl::PathSimilarityMeasurer::Trajectory*> trajs(paths.size());
  for(size_t i = 0; i < paths.size(); ++i)
    trajs[i] = &(paths[i]);

  variance = sbpl::PathSimilarityMeasurer::measureDTW(trajs, 100);
  return true;
}

bool BenchmarkManipulationTests::computePathSimilarity(const std::vector<std::vector<geometry_msgs::Point> > &paths, double &variance, std::vector<std::vector<geometry_msgs::Point> > &paths_compared)
{
  ROS_ERROR("[exp] Function not implemented.");
  return false;
}

bool BenchmarkManipulationTests::writePathSimilarityResultsToFile()
{
  double variance;
  std::string planner;
  std::vector<std::string> descriptions(3);
  descriptions[0] = "simplify"; descriptions[1] = "plan"; descriptions[2] = "interpolate";
  FILE* fptr = NULL;
  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));
  time.erase(time.size()-1, 1);

  std::string filename = "/tmp/path_similarity_stats.csv";
  if((fptr = fopen(filename.c_str(), "a")) == NULL)
  {
    ROS_ERROR("Failed to create consistency file. (%s)", filename.c_str());
    return false;
  }
  ROS_INFO("Successfully created consistency file: %s", filename.c_str());

  fprintf(fptr, "%s, %s, ", time.c_str(), experiment_group_name_.c_str());
  for(size_t i = 0; i < planner_interfaces_.size(); ++i)
  {
    size_t slash = planner_interfaces_[i].find("/");
    planner = planner_interfaces_[i].substr(slash+1);

    fprintf(fptr, "%s, ", planner.c_str());
    for(size_t j = 0; j < descriptions.size(); ++j)
    {
      fprintf(fptr, "%s, ", descriptions[j].c_str());
      for(size_t k = 0; k < fk_link_.size(); ++k)
      {
        if(!computePathSimilarity(planner, "", descriptions[j], fk_link_[k], variance))
          ROS_ERROR("Failed to compute variance for {%s, %s, %s}", planner.c_str(), descriptions[j].c_str(), fk_link_[k].c_str());
        
        fprintf(fptr, "%s, %f, ", fk_link_[k].c_str(), variance);

        if(!computePathSimilarity(planner, "", descriptions[j], fk_link_[k], variance, true))
          ROS_ERROR("Failed to compute variance for {%s, %s, %s}", planner.c_str(), descriptions[j].c_str(), fk_link_[k].c_str());
        
        fprintf(fptr, "%s_DTW, %f, ", fk_link_[k].c_str(), variance);

      }
    }
  }
  fprintf(fptr, "\n");
  fclose(fptr);
  return true;
}

void BenchmarkManipulationTests::visualizeTrajectoryExecution(const trajectory_msgs::JointTrajectory &traj, RobotPose &start, std::string ns, double waypoint_time)
{
  for(size_t k = 0; k < traj.points.size(); ++k)
  {
    RobotPose p = start;
    std::vector<double> base(3,0);
    p.rangles.resize(traj.points[k].positions.size());
    for(size_t q = 0; q < traj.points[k].positions.size(); ++q)
      p.rangles[q] = traj.points[k].positions[q];

    pviz_.visualizeRobot(p.rangles, p.langles, base, p.body.z, 90, ns, 0, true);
    usleep(waypoint_time * 1000000);
  }
}

bool BenchmarkManipulationTests::initPlanners()
{
  std::string urdf, action_set_filename;
  nh_.param<std::string>("robot_description", urdf, " ");
  ph_.param<std::string>("action_set_filename", action_set_filename, "");

  if(urdf.empty() || action_set_filename.empty())
  {
    ROS_ERROR("[exp] The URDF or Action Set Filename not found on param server.");
    return false;
  }

  // distance field
  df_ = new distance_field::PropagationDistanceField(3.0, 3.0, 3.0, 0.02, -0.75, -1.25, -1.0, 0.2);
  df_->reset();

  // robot model
  rm_ = new sbpl_arm_planner::PR2KDLRobotModel();

  if(!rm_->init(urdf, rjoint_names_))
  {
    ROS_ERROR("[exp] Failed to initialize robot model.");
    return false;
  }
  rm_->setPlanningLink("r_gripper_palm_link");
  //rm_->printRobotModelInformation();

  // REMOVE this hardcoded torso-base transform
  KDL::Frame f;
  f.p.x(-0.05); f.p.y(1.0); f.p.z(0.789675);
  f.M = KDL::Rotation::Quaternion(0,0,0,1);
  rm_->setKinematicsToPlanningTransform(f, world_frame_);

  // collision checker
  grid_ = new sbpl_arm_planner::OccupancyGrid(df_);
  grid_->setReferenceFrame(world_frame_);
  cc_ = new sbpl_arm_planner::SBPLCollisionSpace(grid_);

  if(!cc_->init(group_name_))
    return false;
  if(!cc_->setPlanningJoints(rjoint_names_))
    return false;

  // action set
  as_ = new sbpl_arm_planner::ActionSet(action_set_filename);

  // sbpl planner interface
  sbpl_planner_ = new sbpl_arm_planner::SBPLArmPlannerInterface(rm_, cc_, as_, df_);

  if(!sbpl_planner_->init())
  {
    ROS_ERROR("[exp] Failed to initialize the sbpl planner.");
    return false;
  }

  // ompl planner interface
  ompl_planner_ = new sbpl_arm_planner::OMPLArmPlannerInterface(cc_);
  if(!ompl_planner_->init())
  {
    ROS_ERROR("[exp] Failed to initialize the ompl planner.");
    return false;
  }

  ROS_INFO("[exp] Successfully initialized the planners.");
  return true;
}

void BenchmarkManipulationTests::visualizeTrajectory(const trajectory_msgs::JointTrajectory &traj, RobotPose &pose, int throttle, std::string ns, int id)
{
  int path_length = traj.points.size();
  trajectory_msgs::JointTrajectoryPoint pt;
  std::vector<trajectory_msgs::JointTrajectoryPoint> rtraj, ltraj, btraj;

  // right arm
  rtraj = traj.points;

  // left arm
  pt.positions.resize(ljoint_names_.size(),0.0);
  for(size_t i = 0; i < ljoint_names_.size(); ++i)
    pt.positions[i] = pose.langles[i];
  ltraj.resize(path_length, pt);

  // base
  pt.positions.resize(4);
  pt.positions[0] = pose.body.x;
  pt.positions[1] = pose.body.y;
  pt.positions[2] = pose.body.z;
  pt.positions[3] = pose.body.theta;
  btraj.resize(path_length, pt);

  pviz_.visualizeTrajectory(rtraj, ltraj, btraj, throttle, ns, id);
}


