/* \author Benjamin Cohen, Ellis Ratner */

#ifndef _PVIZ_
#define _PVIZ_

#include <string>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <boost/lexical_cast.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <pviz/body_pose.h>

enum{ RIGHT, LEFT, HEAD, TILT};

class PViz
{
  public:

    /* \brief constructor takes in the desired topic name */    
    PViz(const std::string &ns = std::string());

    ~PViz();

    /* \brief set reference frame of all visualization markers */
    void setReferenceFrame(std::string frame) {reference_frame_ = frame;};
    
    /* \brief get reference frame used for the visualizations */
    std::string getReferenceFrame() {return reference_frame_;};

    void getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame);

    void publish(const visualization_msgs::Marker& marker);

    void publish(const visualization_msgs::MarkerArray &marker_array);

    void deleteVisualizations(std::string ns, int max_id);

    bool parseCSVFile(std::string filename, int num_cols, std::vector<std::vector<double> > &data);

    /**************** Robot Meshes ****************/
   
    void visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, double hue, std::string ns, int id, bool use_embedded_materials = false);

    void visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, bool use_embedded_materials = false);

    void visualizeRobotWithTitle(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, std::string title);
 
    bool visualizeTrajectoryFromFile(std::string filename, bool use_embedded_materials = false);

    void visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &rpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &lpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath, int throttle, std::string ns="robot_path", int id=0);

    void visualizeGripper(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, bool open);

    /**************** Shapes, Text & Lines ****************/

    /* \brief visualize a pose (sphere, arrow, string of text) */
    void visualizePose(const std::vector<double> &pose, std::string text);

    /* \brief visualize a pose (sphere, arrow, string of text) */
    void visualizePose(const geometry_msgs::Pose &pose, std::string text);

    /* \brief visualize a pose (sphere, arrow, string of text) */
    void visualizePose(const geometry_msgs::Pose &pose, std::string text, std::string frame_id);

    /* \brief visualize a list of poses (sphere, arrow, pose index number) */
    void visualizePoses(const std::vector<std::vector<double> > &poses);
 
    /* \brief visualize cuboids */
    void visualizeObstacles(const std::vector<std::vector<double> > &obstacles);
   
    void visualize3DPath(std::vector<std::vector<double> > &dpath);

     /* \brief visualize a sphere */
    void visualizeSphere(double x, double y, double z, double radius, int hue, std::string ns, int id);

     /* \brief visualize a sphere */
    void visualizeSphere(std::vector<double> pose, int color, std::string ns, double radius);
    
    /* \brief display a list of spheres of the same radius and color */
    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string ns, double radius);

    /* \brief display a list of spheres of the same color with different radii */
    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string ns, std::vector<double> &radius);
 
    /* \brief display a list of spheres of the same radius and color (radii are 4th element in pose vector) */
    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string ns);
    
    void visualizeSpheres(const std::vector<std::vector<double> > &pose, const std::vector<int> &hue, std::string ns);

    void visualizeSpheres(const std::vector<geometry_msgs::Point>  &poses, int hue, std::string ns, int id, double radius);

    void visualizeLine(const std::vector<geometry_msgs::Point> points, std::string ns, int id, int hue, double thickness);

    void visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, int hue);

    void visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, std::vector<double> color, double size);

    void visualizeText(double x, double y, double z, double size, std::string text, int hue, std::string ns, int id);

    void visualizeCube(geometry_msgs::PoseStamped pose, int hue, std::string ns, int id, std::vector<double> dim);

    /* \brief visualize a mesh where mesh_resource is the path to the mesh using the URI used by resource_retriever package */
    void visualizeMesh(const std::string& mesh_resource, const geometry_msgs::PoseStamped& pose, int color, std::string ns, int id);

    /* \brief visualizes a triangle list; if psychadelic is true then each triangle has one of each red, green, and blue vertices (Gil Jones) */
    void visualizeMeshTriangles(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles, const geometry_msgs::PoseStamped& pose, int color, std::string ns, int id, bool psychadelic);

    /* \brief visualize a list of states (xyz poses) (intended for use with sbpl) */
    void visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size);
    
    /* \brief display a list of states (xyz poses with rpy arrows) (intended for use with sbpl) */
    void visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size);

    /****************** Get Markers *****************/
    void getGripperMeshesMarkerMsg(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, bool open, std::vector<visualization_msgs::Marker> &markers);

    void getGripperMeshesMarkerMsg(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, double position, std::vector<visualization_msgs::Marker> &markers);

    void getCubeMsg(std::vector<double> &cube, std::vector<double> &color, std::string ns, int id, visualization_msgs::Marker& marker);

    void getCubeMsg(geometry_msgs::Pose &pose, std::vector<double> &dim, std::vector<double> &color, std::string ns, int id, visualization_msgs::Marker& marker);

    visualization_msgs::MarkerArray getRobotMeshesMarkerMsg(double hue, std::string ns, int id, std::vector<geometry_msgs::PoseStamped> &poses, bool use_embedded_materials = false);

    visualization_msgs::MarkerArray getRobotMarkerMsg(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, bool use_embedded_materials = false);

  private:

    ros::NodeHandle nh_;
    ros::Publisher marker_array_publisher_;
    ros::Publisher marker_publisher_;

    int num_joints_;

    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker marker_;

    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> arm_link_names_;
    std::vector<std::string> torso_joint_names_;
    std::vector<std::string> torso_link_names_;

    std::vector<std::string> arm_meshes_;
    std::vector<std::string> gripper_meshes_;
    std::vector<std::string> torso_meshes_;
    std::vector<std::string> base_meshes_;
    std::vector<std::string> head_meshes_;
    std::vector<std::string> robot_meshes_;

    KDL::JntArray jnt_pos_in_;
    KDL::JntArray jnt_pos_out_;
    KDL::Frame p_out_;
    KDL::Chain chain_;
    KDL::Tree kdl_tree_;
    std::vector<KDL::ChainFkSolverPos_recursive *> fk_rsolver_;
    std::vector<KDL::ChainFkSolverPos_recursive *> fk_lsolver_;
    KDL::ChainFkSolverPos_recursive * fk_hsolver_;
    KDL::ChainFkSolverPos_recursive * fk_tsolver_;
    std::string reference_frame_;

    /* \brief initialize the KDL chain for the robot arm */
    bool initKDLChain();

    /* \brief compute FK for the pr2 arm meshes using the KDL chain */
    bool computeFKforVisualizationWithKDL(const std::vector<double> &jnt_pos, double torso_pos, int arm, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief compute FK for a joint configuration using the KDL chain */
    bool computeFKwithKDL(const std::vector<double> &angles, std::vector<double> &base_pos, double torso_pos, int arm, int frame_num, geometry_msgs::Pose &pose);

    /* \brief compute FK for all robot meshes */
    bool computeFKforVisualizationWithKDL(const std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief visualize robot meshes...not to be used publicly */
    void visualizeRobotMeshes(double hue, std::string ns, int start_id, std::vector<geometry_msgs::PoseStamped> &poses, bool use_embedded_materials = false);
};

#endif

