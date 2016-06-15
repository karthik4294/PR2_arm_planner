/* \author Ben Cohen */

#include <pviz/pviz.h>
#include <leatherman/viz.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>

static std::string RIGHT_CHAIN_RTIP_NAME = "r_gripper_r_finger_tip_link";
static std::string RIGHT_CHAIN_LTIP_NAME = "r_gripper_l_finger_tip_link";
static std::string LEFT_CHAIN_RTIP_NAME = "l_gripper_r_finger_tip_link";
static std::string LEFT_CHAIN_LTIP_NAME = "l_gripper_l_finger_tip_link";

PViz::PViz(const std::string &ns)
{
  num_joints_ = 8; //arm + torso
  reference_frame_ = "/map";

  srand (time(NULL));
  arm_joint_names_.push_back("_shoulder_pan_joint");
  arm_joint_names_.push_back("_shoulder_lift_joint");
  arm_joint_names_.push_back("_upper_arm_roll_joint");
  arm_joint_names_.push_back("_elbow_flex_joint");
  arm_joint_names_.push_back("_forearm_roll_joint");
  arm_joint_names_.push_back("_wrist_flex_joint");
  arm_joint_names_.push_back("_wrist_roll_joint");

  arm_link_names_.push_back("_shoulder_pan_link");
  arm_link_names_.push_back("_shoulder_lift_link");
  arm_link_names_.push_back("_upper_arm_roll_link");
  arm_link_names_.push_back("_elbow_flex_link");
  arm_link_names_.push_back("_forearm_roll_link");
  arm_link_names_.push_back("_wrist_flex_link");
  arm_link_names_.push_back("_wrist_roll_link");

  torso_link_names_.push_back("torso_lift_link");
  torso_joint_names_.push_back("torso_lift_joint");

  // arm meshes
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_pan.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/upper_arm_roll.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/upper_arm.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/elbow_flex.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/forearm_roll.stl");  
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/forearm.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/wrist_flex.dae");
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/wrist_roll.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/gripper_v0/gripper_palm.dae");

  // gripper meshes
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/l_finger.dae");
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae");
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/l_finger.dae");  
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae");

  // torso_meshes
  torso_meshes_.push_back("package://pr2_description/meshes/torso_v0/torso_lift.dae");
  
  // base meshes
  base_meshes_.push_back("package://pr2_description/meshes/base_v0/base.dae");

  // head meshes
  head_meshes_.push_back("package://pr2_description/meshes/head_v0/head_pan.dae");
  head_meshes_.push_back("package://pr2_description/meshes/head_v0/head_tilt.dae");
  head_meshes_.push_back("package://pr2_description/meshes/tilting_laser_v0/tilting_hokuyo.dae");

  robot_meshes_.insert(robot_meshes_.end(),base_meshes_.begin(), base_meshes_.end());  // 1
  robot_meshes_.insert(robot_meshes_.end(),torso_meshes_.begin(), torso_meshes_.end()); // 1
  robot_meshes_.insert(robot_meshes_.end(),arm_meshes_.begin(), arm_meshes_.end());  // 10
  robot_meshes_.insert(robot_meshes_.end(),gripper_meshes_.begin(), gripper_meshes_.end());  // 4
  robot_meshes_.insert(robot_meshes_.end(),arm_meshes_.begin(), arm_meshes_.end());  // 10
  robot_meshes_.insert(robot_meshes_.end(),gripper_meshes_.begin(), gripper_meshes_.end());  // 4
  robot_meshes_.insert(robot_meshes_.end(),head_meshes_.begin(), head_meshes_.end());  // 3

  if(!initKDLChain())
  {
    ROS_ERROR("[pviz] Failed to initiliaze the KDL chain. This should exit but instead it will crash."); 
    fflush(stdout);
  }

  if(ns.size() > 0)
  {
    std::stringstream ss;
    ss << "/" << ns << "/visualization_marker_array";
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(ss.str(), 500);
    ss.str(std::string());
    ss << "/" << ns << "/visualization_marker";
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(ss.str(), 1000);
  }
  else
  {
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  }
}

PViz::~PViz()
{
  for(size_t i = 0; i < fk_rsolver_.size(); ++i)
    delete fk_rsolver_[i];
  for(size_t i = 0; i < fk_lsolver_.size(); ++i)
    delete fk_lsolver_[i];
}

bool PViz::initKDLChain()
{
  std::string robot_description;
  std::string robot_param = "";
  if(!nh_.searchParam("robot_description",robot_param))
  {
    ROS_ERROR("[pviz] Unable to find the robot_description on the param server.");
    return false;
  }
  nh_.param<std::string>(robot_param,robot_description,"");
  if (!kdl_parser::treeFromString(robot_description, kdl_tree_))
  {
    ROS_ERROR("Failed to parse tree from robot description file.");
    return false;
  }

  // a total of 4 FK solvers are used for both arms & torso
  fk_rsolver_.resize(2);
  fk_lsolver_.resize(2);

  // right arm - right finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", RIGHT_CHAIN_RTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_rsolver_[RIGHT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the right arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //leatherman::printKDLChain("right arm - right finger", chain_);

  // right arm - left finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", RIGHT_CHAIN_LTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_lsolver_[RIGHT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the right arm-left finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //leatherman::printKDLChain("right arm - left finger", chain_);

  // left arm - right finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", LEFT_CHAIN_RTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_rsolver_[LEFT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the left arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //leatherman::printKDLChain("left arm - right finger", chain_);

  // left arm - left finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", LEFT_CHAIN_LTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_lsolver_[LEFT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the left arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //leatherman::printKDLChain("left arm - left finger", chain_);

  jnt_pos_in_.resize(chain_.getNrOfJoints());
  jnt_pos_out_.resize(chain_.getNrOfJoints());
  num_joints_ = chain_.getNrOfJoints();
  ROS_DEBUG("[pviz] jnt_pos_in: rows: %d cols: %d", jnt_pos_in_.rows(), jnt_pos_out_.columns());

  // head (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", "head_tilt_link", chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_hsolver_ = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the head chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //leatherman::printKDLChain("head", chain_);

  // tilt laser (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", "laser_tilt_link", chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the tilt laser. Exiting."); 
    return false;
  }
  fk_tsolver_ = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the tilt laser chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //leatherman::printKDLChain("laser", chain_);

  return true;
}

bool PViz::computeFKwithKDL(const std::vector<double> &angles, std::vector<double> &base_pos, double torso_pos, int arm, int frame_num, geometry_msgs::Pose &pose)
{
  KDL::Frame frame_out;

  // head
  if(arm == HEAD)
  {
    KDL::JntArray head_pos_in;
    head_pos_in.resize(3);
    head_pos_in(0) = torso_pos;
    head_pos_in(1) = 0; // pan
    head_pos_in(2) = 0; // tilt
    if(base_pos.size() > 3)
    {
      head_pos_in(1) = base_pos[3]; // pan
      if(base_pos.size() > 4)
        head_pos_in(2) = base_pos[4]; // tilt
    }

    if(fk_hsolver_->JntToCart(head_pos_in, frame_out, frame_num) < 0)
    {
      ROS_ERROR("JntToCart returned < 0. Exiting. (arm: %d  frame: %d)", arm, frame_num);
      return false;
    }
  }
  else if(arm == TILT) // tilt laser
  {
    KDL::JntArray tilt_pos_in;
    tilt_pos_in.resize(2);
    tilt_pos_in(0) = torso_pos;
    tilt_pos_in(1) = 0;
    if(base_pos.size() > 5)
      tilt_pos_in(1) = base_pos[5]; // tilt mount

    if(fk_tsolver_->JntToCart(tilt_pos_in, frame_out, frame_num) < 0)
    {
      ROS_ERROR("JntToCart returned < 0. Exiting. (arm: %d  frame: %d)", arm, frame_num);
      return false;
    }
  }
  else // body
  {
    jnt_pos_in_(0) = torso_pos;
    for(size_t i = 0; i < angles.size(); ++i)
      jnt_pos_in_(i+1) = angles[i];

    if(angles.size() == 7)
    {
      jnt_pos_in_(8) = 0;
      jnt_pos_in_(9) = 0;
    }
    else if(angles.size() == 8)
    {
      jnt_pos_in_(9) = jnt_pos_in_(8);
    }

    // a negative frame number means that frame is in the left finger chain
    if(frame_num > 0)
    {
      if(fk_rsolver_[arm]->JntToCart(jnt_pos_in_, frame_out, frame_num) < 0)
      {
        ROS_ERROR("JntToCart returned < 0. Exiting. (arm: %d  frame: %d)", arm, frame_num);
        return false;
      }
    }
    else
    {
      if(fk_lsolver_[arm]->JntToCart(jnt_pos_in_, frame_out, -1*frame_num) < 0)
      {
        ROS_ERROR("JntToCart returned < 0. Exiting. (arm: %d  frame: %d)", arm, frame_num);
        return false;
      }
    }
  }

  KDL::Frame map_to_robot;
  getMaptoRobotTransform(base_pos[0],base_pos[1],base_pos[2],map_to_robot);

  frame_out = map_to_robot*frame_out;
  
  pose.position.x = frame_out.p[0];
  pose.position.y = frame_out.p[1];
  pose.position.z = frame_out.p[2];

  frame_out.M.GetQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
  return true;
}

void PViz::visualizeObstacles(const std::vector<std::vector<double> > &obstacles)
{
  visualization_msgs::MarkerArray ma;
  ma.markers.resize(obstacles.size());
  std::vector<double> color(4,0);
  color[2] = 0.5; // light blue
  color[3] = 0.9;

  for(size_t i = 0; i < obstacles.size(); ++i)
    //ma.markers[i] = viz::getCubeMarker(obstacles[i], color, reference_frame_, "obstacles", i);
  
  publish(ma);
}

void PViz::getCubeMsg(std::vector<double> &cube, std::vector<double> &color, std::string ns, int id, visualization_msgs::Marker& marker)
{
  marker = viz::getCubeMarker(cube, color, reference_frame_, ns, id);
}

void PViz::getCubeMsg(geometry_msgs::Pose &pose, std::vector<double> &dim, std::vector<double> &color, std::string ns, int id, visualization_msgs::Marker& marker)
{
  marker = viz::getCubeMarker(pose, dim, color, reference_frame_, ns, id);
}

void PViz::publish(const visualization_msgs::Marker& marker)
{
  //marker_publisher_.publish(marker);

  // instead, publish on the MarkerArray topic
  visualization_msgs::MarkerArray ma;
  ma.markers.push_back(marker);
  publish(ma);
}

void PViz::publish(const visualization_msgs::MarkerArray &marker_array)
{
  marker_array_publisher_.publish(marker_array);
}

void PViz::visualizePoses(const std::vector<std::vector<double> > &poses)
{
  visualization_msgs::MarkerArray ma = viz::getPosesMarkerArray(poses, reference_frame_, "poses", 0);
  publish(ma);
}

void PViz::visualizePose(const std::vector<double> &pose, std::string text)
{
  tf::Quaternion pose_quaternion;
  geometry_msgs::Pose pose_msg;

  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];

  pose_quaternion.setRPY(pose[3],pose[4],pose[5]);
  tf::quaternionTFToMsg(pose_quaternion, pose_msg.orientation);

  visualizePose(pose_msg, text);
}

void PViz::visualizePose(const geometry_msgs::Pose &pose, std::string text)
{
  geometry_msgs::PoseStamped ps;
  ps.pose = pose;
  ps.header.frame_id = reference_frame_;
  visualization_msgs::MarkerArray ma = viz::getPoseMarkerArray(ps, text, 0);
  publish(ma);
}

void PViz::visualizePose(const geometry_msgs::Pose &pose, std::string text, std::string frame_id)
{
  visualization_msgs::MarkerArray ma = viz::getPoseMarkerArray(pose, frame_id, text, 0);
  publish(ma);
}

void PViz::visualizeSphere(std::vector<double> pose, int color, std::string ns, double radius)
{
  visualization_msgs::Marker m = viz::getSphereMarker(pose[0], pose[1], pose[2], radius, color, reference_frame_, ns, 1);
  publish(m);
}

void PViz::visualizeSphere(double x, double y, double z, double radius, int hue, std::string ns, int id)
{
  visualization_msgs::Marker m = viz::getSphereMarker(x, y, z, radius, hue, reference_frame_, ns, 0);
  publish(m);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string ns, double radius)
{
  visualization_msgs::Marker m = viz::getSpheresMarker(pose, radius, color, reference_frame_, ns);
  publish(m);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string ns, std::vector<double> &radius)
{
  visualization_msgs::MarkerArray ma = viz::getSpheresMarkerArray(pose, radius, color, reference_frame_, ns, 0);
  publish(ma);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string ns)
{
  if(pose.empty() || pose[0][3])
    return;

  visualization_msgs::Marker m = viz::getSpheresMarker(pose, pose[0][3], color, reference_frame_, ns, 0);
  publish(m);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, const std::vector<int> &hue, std::string ns)
{
  visualization_msgs::MarkerArray ma = viz::getSpheresMarkerArray(pose, hue, reference_frame_, ns, 0);
  publish(ma);
}

void PViz::deleteVisualizations(std::string ns, int max_id)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(max_id);

  for(int j = 0; j < max_id; j++)
  {
    marker_array_.markers[j].header.stamp = ros::Time::now();
    marker_array_.markers[j].header.frame_id = reference_frame_;
    marker_array_.markers[j].ns = ns;
    marker_array_.markers[j].action = visualization_msgs::Marker::DELETE;
    marker_array_.markers[j].id = j;
  }
  publish(marker_array_);
}

void PViz::visualize3DPath(std::vector<std::vector<double> > &dpath)
{
  if(dpath.empty())
  {
    ROS_WARN("[pviz] The 3D path is empty.");
    return;
  }

  visualization_msgs::Marker obs_marker;
  obs_marker.header.frame_id = reference_frame_;
  obs_marker.header.stamp = ros::Time();
  obs_marker.header.seq = 0;
  obs_marker.ns = "path";
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  obs_marker.action = 0;
  obs_marker.scale.x = 0.06;
  obs_marker.scale.y = 0.06;
  obs_marker.scale.z = 0.06;
  obs_marker.color.r = 0.45;
  obs_marker.color.g = 0.3;
  obs_marker.color.b = 0.4;
  obs_marker.color.a = 0.8;
  obs_marker.lifetime = ros::Duration(0.0);

  obs_marker.points.resize(dpath.size());

  for (int k = 0; k < int(dpath.size()); k++)
  {
    if(int(dpath[k].size()) < 3)
      continue;

    obs_marker.points[k].x = dpath[k][0];
    obs_marker.points[k].y = dpath[k][1];
    obs_marker.points[k].z = dpath[k][2];
  }

  publish(obs_marker);
}

void PViz::visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size)
{
  unsigned int inc = 1;
  visualization_msgs::Marker marker;
  
  //check if the list is empty
  if(states.empty())
  {
    ROS_DEBUG("[pviz] There are no states in the %s states list.", name.c_str());
    return;
  }

  //if there are too many states, rviz will crash and burn when drawing
  if(states.size() > 50000)
    inc = 4;
  else if(states.size() > 10000)
    inc = 1;   //2
  else
    inc = 1;

  marker.points.resize(states.size()/inc + 1);

  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;

  marker.ns = name;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(0.0);

  unsigned int m_ind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
  {
    if(states[i].size() >= 3)
    {
      marker.points[m_ind].x = states[i][0];
      marker.points[m_ind].y = states[i][1];
      marker.points[m_ind].z = states[i][2];
      ++m_ind;
    }
  }

  publish(marker);
  ROS_DEBUG("[visualizeBasicStates] published %d markers for %s states", int(marker.points.size()), name.c_str());
}

void PViz::visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size)
{
  unsigned int inc = 1;
  std::vector<double> scaled_color(4,0);
  visualization_msgs::MarkerArray marker_array;

  //check if the list is empty
  if(states.empty())
  {
    ROS_INFO("[aviz] There are no states in the %s states list", name.c_str());
    return;
  } 
  else
    ROS_INFO("[aviz] There are %i states in the %s states list.",int(states.size()),name.c_str());
    
  if(color.size()<2)
  {
    ROS_INFO("[aviz] Not enough colors specified.");
    return;
  } 
  
  if(color[0].size() < 4 || color[1].size() < 4)
  {
    ROS_INFO("[aviz] RGBA must be specified for each color.");
    return;
  } 
  
  //if there are too many states, rviz will crash and burn when drawing
  /*
  if(states.size() > 50000)
    inc = 20;
  else if(states.size() > 5000)
    inc = 10;
  else if(states.size() > 500)
    inc = 1;   //changed  8/31/11
  else
    inc = 1;
  */

  unsigned int mind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
  {
    marker_array.markers.resize(marker_array.markers.size()+1);
    marker_array.markers[mind].header.frame_id = reference_frame_;
    marker_array.markers[mind].header.stamp = ros::Time::now();
    marker_array.markers[mind].ns = "expanded_states";
    marker_array.markers[mind].id = mind;
    marker_array.markers[mind].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[mind].action =  visualization_msgs::Marker::ADD;
    marker_array.markers[mind].scale.x = size;
    marker_array.markers[mind].scale.y = size;
    marker_array.markers[mind].scale.z = size;
    
    for(unsigned int j = 0; j < 4; ++j)
      scaled_color[j] = color[0][j] - ((color[0][j] - color[1][j]) * (double(i)/double(states.size()/inc)));
      
    marker_array.markers[mind].color.r = scaled_color[0];
    marker_array.markers[mind].color.g = scaled_color[1];
    marker_array.markers[mind].color.b = scaled_color[2];
    marker_array.markers[mind].color.a = 1;
    marker_array.markers[mind].lifetime = ros::Duration(0.0);
    
    marker_array.markers[mind].pose.position.x = states[i][0];
    marker_array.markers[mind].pose.position.y = states[i][1];
    marker_array.markers[mind].pose.position.z = states[i][2];
    
    ++mind;
  } 
  
  ROS_DEBUG("[aviz] published %d markers for %s states", (int)marker_array.markers.size(), name.c_str());
  publish(marker_array);
}

void PViz::visualizeLine(const std::vector<geometry_msgs::Point> points, std::string ns, int id, int hue, double thickness)
{
  visualization_msgs::Marker m = viz::getLineMarker(points, thickness, hue, reference_frame_, ns, id);
  publish(m);
}

void PViz::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, int hue)
{
  visualization_msgs::Marker m = viz::getTextMarker(pose, text, 0.2, hue, reference_frame_, ns, id);
  publish(m);
}

void PViz::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, std::vector<double> color, double size)
{
  visualization_msgs::Marker m = viz::getTextMarker(pose, text, size, color, reference_frame_, ns, id);
  publish(m);
}

void PViz::visualizeCube(geometry_msgs::PoseStamped pose, int hue, std::string ns, int id, std::vector<double> dim)
{
  visualization_msgs::Marker m = viz::getCubeMarker(pose, dim, hue, ns, id);
  publish(m);
}

void PViz::visualizeMesh(const std::string& mesh_resource, const geometry_msgs::PoseStamped& pose, int color, std::string ns, int id)
{
  visualization_msgs::Marker m = viz::getMeshMarker(pose, mesh_resource, color, ns, id);
  publish(m);
}

void PViz::visualizeMeshTriangles(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles, const geometry_msgs::PoseStamped& pose, int color, std::string ns, int id, bool psychadelic)
{
  visualization_msgs::Marker m = viz::getMeshMarker(pose, vertices, triangles, color, psychadelic, ns, id);
  publish(m);
}

bool PViz::computeFKforVisualizationWithKDL(const std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, std::vector<geometry_msgs::PoseStamped> &poses)
{
  //output is PoseStamped only so it has same interface as computeFKforVisualization
  //KDL::Frame map_to_robot; 
  geometry_msgs::Pose pose;

/*
[ INFO]:   frame  0: segment: 0.000 0.000 0.051  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  1: segment: -0.050 0.000 0.740  joint: -0.050 0.000 0.740   joint_type: TransAxis
[ INFO]:   frame  2: segment: 0.000 0.188 0.000  joint: 0.000 0.188 0.000   joint_type: RotAxis
[ INFO]:   frame  3: segment: 0.100 0.000 0.000  joint: 0.100 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  4: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  5: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  6: segment: 0.400 0.000 0.000  joint: 0.400 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  7: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  8: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  9: segment: 0.321 0.000 0.000  joint: 0.321 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame 10: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame 11: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame 12: segment: 0.077 0.010 0.000  joint: 0.077 0.010 0.000   joint_type: RotAxis
[ INFO]:   frame 13: segment: 0.091 0.005 0.000  joint: 0.091 0.005 0.000   joint_type: RotAxis

  0 base_v0/base.stl
  1 torso_v0/torso_lift.stl
  2 shoulder_v0/shoulder_yaw.stl
  3 shoulder_v0/shoulder_lift.stl
  4 upper_arm_roll
  5 upper_arm_v0/upper_arm.stl
  6 upper_arm_v0/elbow_flex.stl
  7 upper_arm_v0/forearm_roll.stl
  8 forearm_v0/forearm.stl
  9 forearm_v0/wrist_flex.stl
  10 forearm_v0/wrist_roll.stl
  11 gripper_v0/gripper_palm.stl
  12 gripper_v0/upper_finger_r.stl
  13 gripper_v0/finger_tip_r.stl
 
  12 gripper_v0/upper_finger_l.stl
  13 gripper_v0/finger_tip_l.stl
  

  2 shoulder_v0/shoulder_yaw.stl
  3 shoulder_v0/shoulder_lift.stl
  4 upper_arm_roll
  5 upper_arm_v0/upper_arm.stl
  6 upper_arm_v0/elbow_flex.stl
  7 upper_arm_v0/forearm_roll.stl
  8 forearm_v0/forearm.stl
  9 forearm_v0/wrist_flex.stl
  10 forearm_v0/wrist_roll.stl
  11 gripper_v0/gripper_palm.stl
  12 gripper_v0/upper_finger_r.stl
  13 gripper_v0/finger_tip_r.stl
  12 gripper_v0/upper_finger_l.stl
  13 gripper_v0/finger_tip_l.stl
 
  30 meshes total

*/

  //getMapToRobotTransform(base_pos[0],base_pos[1],base_pos[2],map_to_robot)

  poses.resize(robot_meshes_.size());
  for(int i=0; i <  int(poses.size()); i++)
  {
    // base, torso, right arm thru the right finger
    if(i < 14)
    {
      if(!computeFKwithKDL(jnt0_pos, base_pos, torso_pos, RIGHT, i+1, poses[i].pose))
        return false;

      // right arm, right finger (to rotate the mesh)
      if(i == 13 || i == 12)
      {
        geometry_msgs::Pose p;
        p.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
        leatherman::multiply(poses[i].pose, p, poses[i].pose);
      }
    }
    // right arm - left finger only
    else if(13 < i && i < 16)
    {
      if(!computeFKwithKDL(jnt0_pos, base_pos, torso_pos, RIGHT, -1*(i-1), poses[i].pose))
        return false;
    }
    // left arm thru the right finger
    else if(15 < i && i < 28)
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, LEFT, i-13, poses[i].pose))
        return false;

      if(i == 26 || i == 27)
      {
        geometry_msgs::Pose p;
        p.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
        leatherman::multiply(poses[i].pose, p, poses[i].pose);
      }
    }
    // left arm - left finger only
    else if(27 < i && i < 30) 
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, LEFT, -1*(i-15), poses[i].pose))
        return false;
    }
    // head pan, head tilt
    else if(29 < i && i < 32)
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, HEAD, i-27, poses[i].pose))
        return false;
    }
    // tilt laser
    else if(31 < i)
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, TILT, i-29, poses[i].pose))
        return false;
    }
  }

  return true;
}

void PViz::visualizeRobotMeshes(double hue, std::string ns, int id, std::vector<geometry_msgs::PoseStamped> &poses, bool use_embedded_materials)
{
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(robot_meshes_.size());
  ros::Time time = ros::Time();

  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].header.stamp = time;
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = ns;
    marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array_.markers[i].id = id + i;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose = poses.at(i).pose;
    marker_array_.markers[i].scale.x = 1.0;
    marker_array_.markers[i].scale.y = 1.0;
    marker_array_.markers[i].scale.z = 1.0;
    if(use_embedded_materials)
    {
      marker_array_.markers[i].color.r = 0;
      marker_array_.markers[i].color.g = 0;
      marker_array_.markers[i].color.b = 0;
      marker_array_.markers[i].color.a = 0;
      marker_array_.markers[i].mesh_use_embedded_materials = true;
    }
    else
    {
      marker_array_.markers[i].color.r = r;
      marker_array_.markers[i].color.g = g;
      marker_array_.markers[i].color.b = b;
      marker_array_.markers[i].color.a = 0.4;
    }
    marker_array_.markers[i].lifetime = ros::Duration(0.0);
    marker_array_.markers[i].mesh_resource = robot_meshes_[i];
  }
  publish(marker_array_);
}

visualization_msgs::MarkerArray PViz::getRobotMeshesMarkerMsg(double hue, std::string ns, int id, std::vector<geometry_msgs::PoseStamped> &poses, bool use_embedded_materials)
{
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(robot_meshes_.size());
  ros::Time time = ros::Time();
  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].header.stamp = time;
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = ns;
    marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array_.markers[i].id = id + i;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose = poses.at(i).pose;
    marker_array_.markers[i].scale.x = 1.0;
    marker_array_.markers[i].scale.y = 1.0;
    marker_array_.markers[i].scale.z = 1.0;
    if(use_embedded_materials)
    {
      marker_array_.markers[i].color.r = 0;
      marker_array_.markers[i].color.g = 0;
      marker_array_.markers[i].color.b = 0;
      marker_array_.markers[i].color.a = 0;
      marker_array_.markers[i].mesh_use_embedded_materials = true;
    }
    else
    {
      marker_array_.markers[i].color.r = r;
      marker_array_.markers[i].color.g = g;
      marker_array_.markers[i].color.b = b;
      marker_array_.markers[i].color.a = 0.4;
    }
    marker_array_.markers[i].lifetime = ros::Duration(0.0);
    marker_array_.markers[i].mesh_resource = robot_meshes_[i];
  }
  return marker_array_;
}

void PViz::getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame)
{
  KDL::Rotation r1;
  r1.DoRotZ(theta);
  KDL::Vector t1(x,y,0.0);
  KDL::Frame base_footprint_in_map(r1,t1);
  frame = base_footprint_in_map;
}

void PViz::visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, double hue, std::string ns, int id, bool use_embedded_materials)
{
  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualizationWithKDL(jnt0_pos, jnt1_pos, base_pos, torso_pos, poses))

    ROS_WARN("Unable to compute forward kinematics.");
  else
    visualizeRobotMeshes(hue, ns, id, poses, use_embedded_materials);
}

void PViz::visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, bool use_embedded_materials)
{
  double torso_pos;
  std::vector<double> base_pos(3,0);

  base_pos[0] = body_pos.x;
  base_pos[1] = body_pos.y;
  base_pos[2] = body_pos.theta;
  torso_pos = body_pos.z;

  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualizationWithKDL(jnt0_pos, jnt1_pos, base_pos, torso_pos, poses))

    ROS_WARN("Unable to compute forward kinematics.");
  else
    visualizeRobotMeshes(hue, ns, id, poses, use_embedded_materials);
}

visualization_msgs::MarkerArray PViz::getRobotMarkerMsg(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, bool use_embedded_materials)
{
  double torso_pos;
  std::vector<double> base_pos(3,0);

  base_pos[0] = body_pos.x;
  base_pos[1] = body_pos.y;
  base_pos[2] = body_pos.theta;
  torso_pos = body_pos.z;

  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualizationWithKDL(jnt0_pos, jnt1_pos, base_pos, torso_pos, poses))
  {
    ROS_WARN("[pviz] Failed to compute forward kinematics. Cannot visualize robot.");
    return visualization_msgs::MarkerArray();
  } 
  else
    return getRobotMeshesMarkerMsg(hue, ns, id, poses, use_embedded_materials);
}

bool PViz::parseCSVFile(std::string filename, int num_cols, std::vector<std::vector<double> > &data)
{
  // NOTE: This function requires that every line ends with a comma as well.
  std::ifstream input_file(filename.c_str());
  if(!input_file.good())
  {
    printf("[pviz] Unable to open '%s' for reading.\n",filename.c_str());
    return false;
  }

  int row(0), col(0);
  char line[256];
  input_file.seekg(0);

  std::vector<std::vector<double> > raw_data;

  row = -1;
  raw_data.clear();

  while (input_file.getline(line, 256, ','))
  {
    raw_data.resize(raw_data.size()+1);
    raw_data[++row].resize(num_cols);
    raw_data[row][0] = atof(line);

    for(col = 1; col < num_cols; col++)
    {
      input_file.getline(line, 256, ',');
      raw_data[row][col] = atof(line);
    }
  }

  std::vector<double> zero_line(num_cols,0);
  for(int i = 0; i < int(raw_data.size()); i++)
  {
    if(raw_data[i] != zero_line)
      data.push_back(raw_data[i]);
  }

  ROS_DEBUG("[pviz] raw_data: num rows: %d",(int)raw_data.size());
  ROS_DEBUG("[pviz] data: num rows: %d num_cols: %d",(int)data.size(), int(data[0].size()));
  return true;
}

bool PViz::visualizeTrajectoryFromFile(std::string filename, bool use_embedded_materials)
{
  double torso = 0;
  std::vector<double> jnt0_pos(7,0), jnt1_pos(7,0), base_pos(3,0);
  std::vector<std::vector<double> > traj;
  if(!parseCSVFile(filename, 18, traj))
  {
    ROS_ERROR("[pviz] Failed to parse trajectory file: %s", filename.c_str());
    return false;
  }

  for(size_t i = 0; i < traj.size(); ++i)
  {
    for(size_t j = 0; j < 7; ++j)
    {
      jnt0_pos[j] = traj[i][4+j];
      jnt1_pos[j] = traj[i][12+j];
    }

    base_pos[0] = traj[i][0];
    base_pos[1] = traj[i][1];
    base_pos[2] = traj[i][2];
    torso = traj[i][3];

    ROS_INFO("[pviz] %d: visualizing robot...", int(i));
    visualizeRobot(jnt0_pos,jnt1_pos,base_pos,torso,(30*i)%300,"full_body_waypoint_"+boost::lexical_cast<std::string>(i),i, use_embedded_materials);
    ros::spinOnce();
    usleep(10000);
  }
  return true;
}

void PViz::visualizeRobotWithTitle(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, std::string title)
{
  visualizeRobot(jnt0_pos, jnt1_pos, body_pos, hue, ns, id);

  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns + "_title";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.pose.position.x = body_pos.x;
  marker.pose.position.y = body_pos.y;
  marker.pose.position.z = body_pos.z+1.5;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.5;
  marker.text = title;
  marker.lifetime = ros::Duration(0.0);
  publish(marker);
}

void PViz::visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath, int throttle, std::string ns, int id)
{
  int length = rpath.size();
  std::vector<double> rangles(7, 0), langles(7, 0);
  BodyPose body_pos;
  visualization_msgs::MarkerArray ma, ma1;

  if (rpath.size() != lpath.size() || rpath.size() != bpath.size()) {
    ROS_ERROR("[pviz] The right arm, left arm and body trajectories are of unequal lengths.");
    return;
  }

  int color_inc = 240.0 / (length / throttle); // hue: red -> blue
  ROS_DEBUG("[pviz] length: %d color_inc: %d throttle: %d)", length, color_inc, throttle);

  for (int i = 0; i < length; ++i) {
    for (std::size_t j = 0; j < rangles.size(); ++j) {
      rangles[j] = rpath[i].positions[j];
      langles[j] = lpath[i].positions[j];
    }
    body_pos.x = bpath[i].positions[0];
    body_pos.y = bpath[i].positions[1];
    body_pos.z = bpath[i].positions[2];
    body_pos.theta = bpath[i].positions[3];

    if ((i != length - 1) && (i % throttle != 0))
      continue;

    ROS_DEBUG("[pviz] length: %d color_inc: %d throttle: %d", length, color_inc, throttle);
    ROS_DEBUG("[pviz] Visualizing waypoint #%d (i mod color_inc: %d) with color: %d (color_inc: %d, throttle: %d)", i, (i / throttle), (i / throttle) * color_inc, color_inc, throttle);

    ma1 = getRobotMarkerMsg(rangles, langles, body_pos, (i / throttle) * color_inc, ns, id+(i+1)*30);
    ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end()); 
  }
  ROS_INFO("[pviz] Visualizing a robot path with %d waypoints. (throttle = %d)", int(rpath.size()), throttle);
  publish(ma);
}

void PViz::visualizeSpheres(const std::vector<geometry_msgs::Point>  &poses, int hue, std::string ns, int id, double radius)
{
  visualization_msgs::Marker m = viz::getSpheresMarker(poses, radius, hue, reference_frame_, ns, id);
  publish(m);
}

void PViz::visualizeGripper(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, bool open)
{
  visualization_msgs::MarkerArray m;
  getGripperMeshesMarkerMsg(pose, hue, ns, id, open, m.markers);
  publish(m);
}

void PViz::getGripperMeshesMarkerMsg(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, bool open, std::vector<visualization_msgs::Marker> &markers)
{
  static geometry_msgs::Pose rot;
  rot.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
  static geometry_msgs::Pose g1, g2, g3, g4, g5, g6, g7, g8;
  // g1 = r_gripper_r_finger_link in r_gripper_palm - OPEN
  g1.position.x = 0.077; g1.position.y = -0.010; g1.position.z = 0.00;
  g1.orientation.x = 0.0; g1.orientation.y = 0.0; g1.orientation.z = -0.231; g1.orientation.w = 0.973;
  leatherman::multiply(g1, rot, g1);
  // g2 = r_gripper_l_finger_link in r_gripper_palm - OPEN
  g2.position.x = 0.077; g2.position.y = 0.010; g2.position.z = 0.00;
  g2.orientation.x = 0.0; g2.orientation.y = 0.0; g2.orientation.z = 0.231; g2.orientation.w = 0.973;
  // g2 = g1; g2.position.y = 0.010; g2.orientation.z = 0.231;
  // g3 = r_gripper_r_finger_tip_link in r_gripper_palm - OPEN
  g3.position.x = 0.156; g3.position.y = -0.056; g3.position.z = 0.00;
  g3.orientation.x = 0.0; g3.orientation.y = 0.0; g3.orientation.z = 0.0; g3.orientation.w = 1.000;
  leatherman::multiply(g3, rot, g3);
  // g4 = r_gripper_l_finger_tip_link in r_gripper_palm - OPEN
  g4.position.x = 0.156; g4.position.y = 0.056; g4.position.z = 0.00;
  g4.orientation.x = 0.0; g4.orientation.y = 0.0; g4.orientation.z = 0.0; g4.orientation.w = 1.000;
  // g4 = g3; g4.position.y = 0.056;
  // g5 = r_gripper_r_finger_link in r_gripper_palm - CLOSED
  g5.position.x = 0.077; g5.position.y = -0.010; g5.position.z = 0.00;
  g5.orientation.x = 0.0; g5.orientation.y = 0.0; g5.orientation.z = 0.004; g5.orientation.w = 1.000;
  // g5 = g1; g5.orientation.x = 0.0; g5.orientation.y = 0.0; g5.orientation.z = 0.004; g5.orientation.w = 1.000;
  leatherman::multiply(g5, rot, g5);
  // g6 = r_gripper_l_finger_link in r_gripper_palm - CLOSED
  g6.position.x = 0.077; g6.position.y = 0.010; g6.position.z = 0.00;
  g6.orientation.x = 0.0; g6.orientation.y = 0.0; g6.orientation.z = -0.004; g6.orientation.w = 1.000;
  // g6 = g5; g6.position.y = 0.010; g6.orientation.z = -0.004;
  // g7 = r_gripper_r_finger_tip_link in r_gripper_palm - CLOSED
  g7.position.x = 0.168; g7.position.y = -0.014; g7.position.z = 0.00;
  g7.orientation.x = 0.0; g7.orientation.y = 0.0; g7.orientation.z = 0.0; g7.orientation.w = 1.000;
  leatherman::multiply(g7, rot, g7);
  // g8 = r_gripper_l_finger_tip_link in r_gripper_palm - CLOSED
  g8.position.x = 0.168; g8.position.y = 0.014; g8.position.z = 0.00;
  g8.orientation.x = 0.0; g8.orientation.y = 0.0; g8.orientation.z = 0.0; g8.orientation.w = 1.000;
  // g8 = g7; g8.position.y = 0.014;

  double r,g,b;
  visualization_msgs::Marker m;
  std::vector<geometry_msgs::Pose> p(4);

  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  m.header.stamp = ros::Time::now();
  m.header.frame_id = "base_footprint";
  m.ns = ns;
  m.type = visualization_msgs::Marker::MESH_RESOURCE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0.0);

  if(open)
  {
    p[0] = g1;
    p[1] = g2;
    p[2] = g3;
    p[3] = g4;
  }
  else
  {
    p[0] = g5;
    p[1] = g6;
    p[2] = g7;
    p[3] = g8;
  }

  // palm
  m.mesh_resource = arm_meshes_[9];
  m.id = id;
  m.pose = pose;
  markers.push_back(m);

  // upper_finger_r
  m.mesh_resource = gripper_meshes_[0];
  m.id++;
  leatherman::multiply(pose, p[0], m.pose);
  markers.push_back(m);

  // upper_finger_l
  m.mesh_resource = gripper_meshes_[2];
  m.id++;
  leatherman::multiply(pose, p[1], m.pose);
  markers.push_back(m);

  m.color.r = 90.0/255.0;
  m.color.g = 90.0/255.0;
  m.color.b = 90.0/255.0;

  // finger_tip_r
  m.mesh_resource = gripper_meshes_[1];
  m.id++;
  leatherman::multiply(pose, p[2], m.pose);
  markers.push_back(m);

  // finger_tip_l
  m.mesh_resource = gripper_meshes_[3];
  m.id++;
  leatherman::multiply(pose, p[3], m.pose);
  markers.push_back(m);
}

void PViz::getGripperMeshesMarkerMsg(const geometry_msgs::Pose &pose, double hue, 
				     std::string ns, int id, double position, 
				     std::vector<visualization_msgs::Marker> &markers)
{
  static geometry_msgs::Pose rot;
  rot.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
  // r_gripper_r_finger_link in r_gripper_palm
  static geometry_msgs::Pose rgrfl_in_rgp;
  rgrfl_in_rgp.position.x = 0.077;
  rgrfl_in_rgp.position.y = -0.010;
  rgrfl_in_rgp.position.z = 0.00;
  rgrfl_in_rgp.orientation.x = 0.0;
  rgrfl_in_rgp.orientation.y = 0.0;
  rgrfl_in_rgp.orientation.z = 0.004;
  rgrfl_in_rgp.orientation.w = 1.000;
  leatherman::multiply(rgrfl_in_rgp, rot, rgrfl_in_rgp);

  // r_gripper_l_finger_link in r_gripper_palm
  static geometry_msgs::Pose rglfl_in_rgp;
  rglfl_in_rgp.position.x = 0.077;
  rglfl_in_rgp.position.y = 0.010;
  rglfl_in_rgp.position.z = 0.00;
  rglfl_in_rgp.orientation.x = 0.0;
  rglfl_in_rgp.orientation.y = 0.0;
  rglfl_in_rgp.orientation.z = -0.004;
  rglfl_in_rgp.orientation.w = 1.000;
    
  // r_gripper_r_finger_tip_link in r_gripper_r_finger_link
  static geometry_msgs::Pose rgrftl_in_rgrfl;
  rgrftl_in_rgrfl.position.x = 0.091;
  rgrftl_in_rgrfl.position.y = -0.005;
  rgrftl_in_rgrfl.position.z = 0.000;
  rgrftl_in_rgrfl.orientation.x = 0.000;
  rgrftl_in_rgrfl.orientation.y = 0.000;
  rgrftl_in_rgrfl.orientation.z = 0.001;
  rgrftl_in_rgrfl.orientation.w = 1.000;

  // r_gripper_l_finger_tip_link in r_gripper_l_finger_link
  static geometry_msgs::Pose rglftl_in_rglfl;
  rglftl_in_rglfl.position.x = 0.091;
  rglftl_in_rglfl.position.y = 0.005;
  rglftl_in_rglfl.position.z = 0.000;
  rglftl_in_rglfl.orientation.x = 0.000;
  rglftl_in_rglfl.orientation.y = 0.000;
  rglftl_in_rglfl.orientation.z = -0.001;
  rglftl_in_rglfl.orientation.w = 1.000;

  // Open the gripper appropriately.
  geometry_msgs::Pose open;
  open.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, position);
  leatherman::multiply(rgrfl_in_rgp, open, rgrfl_in_rgp);
  leatherman::multiply(rglfl_in_rgp, open, rglfl_in_rgp);

  open.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -position);
  leatherman::multiply(rgrftl_in_rgrfl, open, rgrftl_in_rgrfl);
  leatherman::multiply(rglftl_in_rglfl, open, rglftl_in_rglfl);

  geometry_msgs::Pose rgrftl_in_rgp;
  leatherman::multiply(rgrfl_in_rgp, rgrftl_in_rgrfl, rgrftl_in_rgp);
  geometry_msgs::Pose rglftl_in_rgp;
  leatherman::multiply(rglfl_in_rgp, rglftl_in_rglfl, rglftl_in_rgp);

  double r, g, b;
  visualization_msgs::Marker m;
  std::vector<geometry_msgs::Pose> p(4);
  p[0] = rgrfl_in_rgp;
  p[1] = rglfl_in_rgp;
  p[2] = rgrftl_in_rgp;
  p[3] = rglftl_in_rgp;

  leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  m.header.stamp = ros::Time::now();
  m.header.frame_id = "/base_footprint";
  m.ns = ns;
  m.type = visualization_msgs::Marker::MESH_RESOURCE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;

  // palm
  m.mesh_resource = arm_meshes_[9];
  m.id = id;
  m.pose = pose;
  markers.push_back(m);

  // upper_finger_r
  m.mesh_resource = gripper_meshes_[0];
  m.id++;
  leatherman::multiply(pose, p[0], m.pose);
  markers.push_back(m);

  // upper_finger_l
  m.mesh_resource = gripper_meshes_[2];
  m.id++;
  leatherman::multiply(pose, p[1], m.pose);
  markers.push_back(m);

  m.color.r = 90.0/255.0;
  m.color.g = 90.0/255.0;
  m.color.b = 90.0/255.0;

  // finger_tip_r
  m.mesh_resource = gripper_meshes_[1];
  m.id++;
  leatherman::multiply(pose, p[2], m.pose);
  markers.push_back(m);

  // finger_tip_l
  m.mesh_resource = gripper_meshes_[3];
  m.id++;
  leatherman::multiply(pose, p[3], m.pose);
  markers.push_back(m);
}

void PViz::visualizeText(double x, double y, double z, double size, std::string text, int hue, std::string ns, int id)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;
  visualization_msgs::Marker m = viz::getTextMarker(p, text, size, hue, reference_frame_, ns, id);
  publish(m);
}

