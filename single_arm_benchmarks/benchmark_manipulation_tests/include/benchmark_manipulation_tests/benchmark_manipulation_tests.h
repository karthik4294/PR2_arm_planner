#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <iterator>
#include <list>
#include <map>
#include <sstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <pviz/pviz.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>
#include <leatherman/file.h>
#include <sbpl_geometry_utils/PathSimilarityMeasurer.h>

#include <ompl_arm_planner/ompl_arm_planner_interface.h>
#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>
#include <sbpl_manipulation_components/collision_checker.h>
#include <sbpl_manipulation_components_pr2/pr2_kdl_robot_model.h>
#include <benchmark_manipulation_tests/GetMobileArmPlan.h>

#include <sbpl/headers.h>

typedef struct
{
  std::vector<double> langles;
  std::vector<double> rangles;
  BodyPose body;
} RobotPose;

typedef struct
{
  int pre_action;  // 0:nothing, 1:attach, 2:detach
  int post_action; // 0:nothing, 1:attach, 2:detach

  std::string name;
  std::string goal;
  std::string sound_bite;
  RobotPose start;
} Experiment;

class BenchmarkManipulationTests
{
  public:
    
    BenchmarkManipulationTests();
    ~BenchmarkManipulationTests(){};

    bool getParams();
    bool getLocations();
    bool getExperiments();
    bool initPlanners();
  
    void printLocations();
    void printExperiments();
    void printParams();
    void printRobotPose(RobotPose &pose, std::string name);
    void startCompleteExperimentFile();
    void writeCompleteExperimentFile(Experiment e, RobotPose start);

    bool getCollisionObjects(std::string filename, std::vector<moveit_msgs::CollisionObject> &collision_objects);
    bool getCollisionObjects(std::string filename, std::vector<arm_navigation_msgs::CollisionObject> &collision_objects);
    void removeCollisionObject(std::string id);
    bool getAttachedObject(std::string object_file, geometry_msgs::Pose rarm_object_pose, moveit_msgs::AttachedCollisionObject &att_object);
    bool getAttachedObject(std::string object_file, geometry_msgs::Pose rarm_object_pose, arm_navigation_msgs::AttachedCollisionObject &att_object);

    bool requestPlan(RobotPose &start_state, std::string name);
    bool performAllExperiments();
    void writeStatsOMPL(double time);
    bool runExperiment(std::string name);

    void visualizeLocations();
    void visualizeRobotPose(RobotPose &pose, std::string name, int id);
    void visualizeStartPose();
    void visualizeEnvironment();
    void visualizePaths();
    void pathtoVisualizationMarker(visualization_msgs::MarkerArray & path_array, std::vector<double> coord, int id);
    void visualizeTrajectory(const trajectory_msgs::JointTrajectory &traj, RobotPose &pose, int throttle, std::string ns, int id=0);
    void visualizeTrajectoryExecution(const trajectory_msgs::JointTrajectory &traj, RobotPose &start, std::string ns, double waypoint_time);

    void fillSingleArmPlanningRequest(RobotPose &start_state, std::string name, moveit_msgs::MotionPlanRequest &req);
    void fillSingleArmPlanningRequest(RobotPose &start_state, std::string name, arm_navigation_msgs::MotionPlanRequest &req);

    bool setRobotPoseFromTrajectory(moveit_msgs::RobotTrajectory &trajectory, moveit_msgs::RobotState &trajectory_start, RobotPose &pose);
    bool setRobotPoseFromTrajectory(trajectory_msgs::JointTrajectory &trajectory, arm_navigation_msgs::RobotState &trajectory_start, RobotPose &pose);
    bool getRobotPoseFromRobotState(const arm_navigation_msgs::RobotState &state, RobotPose &pose);
    bool getRobotPoseFromRobotState(const moveit_msgs::RobotState &state, RobotPose &pose);
    bool getBasePoseFromPlanningScene(const moveit_msgs::PlanningScene &scene, BodyPose &body);
    bool getJointPositionsFromTrajectory(const trajectory_msgs::JointTrajectory &traj, std::vector<std::string> &names, int waypoint, std::vector<double> &angles);

    bool computeFK(std::vector<double> &angles, std::vector<geometry_msgs::Pose> &pose);
    bool printPathToFile(FILE** file, const trajectory_msgs::JointTrajectory &traj);
    bool createTrajectoryFile(std::string exp_name, std::string planner, std::string planner_id, std::string description, FILE** file, std::string &filename);
    bool createTrajectoryFolder(std::string exp_name);
    bool writeTrajectoriesToFile(const std::vector<trajectory_msgs::JointTrajectory> &trajs, std::vector<std::string> &planners, std::vector<std::string> &planner_ids, std::vector<std::string> &descriptions, std::string exp_name);
    bool createFolder(std::string name);
    bool createExperimentGroupFolder(std::string exp_group_name);
    bool writeStatsToFile(const std::vector<trajectory_msgs::JointTrajectory> &trajs, std::vector<std::string> &planners, std::vector<std::string> &planner_ids, std::vector<std::string> &descriptions, std::map<std::string, std::vector<double> > &stats, std::string exp_name);

    bool getPathFromFile(std::string filename, trajectory_msgs::JointTrajectory &traj, std::vector<std::vector<geometry_msgs::Pose> > &poses);
    bool getPathFromFile(std::string filename, std::vector<std::vector<double> > &path);
    void printTrajectoryFilenames();
    bool computePathSimilarity(std::string planner_interface, std::string planner_id, std::string description, std::string link_name, double &variance, bool use_dtw=false);
    bool computePathSimilarity(const std::vector<std::vector<geometry_msgs::Point> > &paths, double &variance);
    bool computePathSimilarityDTW(const std::vector<std::vector<geometry_msgs::Point> > &paths, double &variance);
    bool writePathSimilarityResultsToFile();
    bool getLinkPaths(std::string planner_interface, std::string planner_id, std::string description, std::string link_name, std::vector<std::vector<geometry_msgs::Point> > &paths, bool visualize=false, int hue=10);
    bool computePathSimilarity(const std::vector<std::vector<geometry_msgs::Point> > &paths, double &variance, std::vector<std::vector<geometry_msgs::Point> > &paths_compared);

  private:
    benchmark_manipulation_tests::GetMobileArmPlan::Request req2;
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher attached_object_pub_;
    ros::Publisher collision_object_pub_;
    ros::Publisher pscene_pub_;
    ros::Publisher ma_pub_;
    ros::Publisher path_marker_;
    PViz pviz_;

    std::map<std::string, Experiment> exp_map_;
    std::map<std::string, int> action_map_;
    std::map<std::string, std::vector<double> > loc_map_;
    std::vector<std::string> action_list_;
    RobotPose start_pose_; 
    RobotPose current_pose_; 

    std::vector<std::string> planner_interfaces_;
    std::vector<std::string> rjoint_names_;
    std::vector<std::string> ljoint_names_;
    std::vector<double> goal_tolerance_;
    std::vector<double> rarm_object_offset_;
    std::vector<double> larm_object_offset_;
    std::vector<double> collision_object_offset_;
    geometry_msgs::Pose rarm_object_pose_;
    geometry_msgs::Pose larm_object_pose_;
    geometry_msgs::Pose collision_object_offset_pose_;
    bool apply_offset_to_collision_objects_;

    std::string known_objects_filename_;
    std::string attached_object_filename_;
    std::string benchmark_results_folder_;
    std::string trajectory_folder_name_;
    std::string trajectory_folder_path_;
    std::string trajectory_files_path_;
    std::string sbpl_planner_id_;
    std::string ompl_planner_id_;
    std::string eps_file_;
    std::string timeout_file_;
    std::string environment_name_;
    int exp_run_count_;

    bool use_current_state_as_start_;
    bool use_initial_start_as_every_start_;
    int experiment_type_; // 1: one arm, 2: two arm
    int average_count_;
    std::string group_name_;
    std::string planner_id_;
    std::string benchmark_service_name_;
    std::string spine_frame_;
    std::string world_frame_;
    std::string robot_model_root_frame_;
    std::string experiment_group_name_;
    std::string display_trajectory_description_;
    std::vector<std::string> fk_link_;

    std::vector<double> next_start_;

    // added in last minute for parsing code to be a touch nicer
    int num_joints_;
    std::map<std::string, std::vector<std::vector<std::string> > > trajectory_file_map_;
  
    // planners
    distance_field::PropagationDistanceField *df_;
    sbpl_arm_planner::RobotModel *rm_;
    sbpl_arm_planner::OccupancyGrid *grid_;
    sbpl_arm_planner::ActionSet *as_;
    sbpl_arm_planner::SBPLCollisionSpace *cc_;
    sbpl_arm_planner::SBPLArmPlannerInterface *sbpl_planner_;
    sbpl_arm_planner::OMPLArmPlannerInterface *ompl_planner_;

    //SPI
    ompl::base::SpaceInformationPtr si_;

    visualization_msgs::MarkerArray sg_;
    ros::Publisher sg_pub_;

    ompl::RNG* rng_;
};

