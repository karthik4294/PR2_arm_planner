/*
 * Copyright (c) 2011, Maxim Likhachev
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

/** \author Mike Phillips (rearranged by Benjamin Cohen)  */

#ifndef _OMPL_ARM_PLANNER_INTERFACE_
#define _OMPL_ARM_PLANNER_INTERFACE_

#include <iostream>
//#include <map>
#include <ros/ros.h>
//#include <boost/thread/mutex.hpp>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>
//#include <message_filters/subscriber.h>
//#include <tf/message_filter.h>
//#include <tf/transform_datatypes.h>
//#include <kdl/chain.hpp>
//#include <kdl/frames.hpp>
//#include <angles/angles.h>
#include <sbpl_manipulation_components/collision_checker.h>

/** Messages **/
//#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

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
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include "ompl/base/goals/GoalState.h"


class OMPLArmCollisionChecker : public ompl::base::StateValidityChecker
{
  public:

    OMPLArmCollisionChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si){}

    void initialize(sbpl_arm_planner::CollisionChecker* c)
    {
      cc_ = c;
      num = new int[4];
    }

    virtual bool isValid(const ompl::base::State *state) const
    {
      const ompl::base::RealVectorStateSpace::StateType* s = dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*> (state);

      std::vector<double> pose(7,0);
      pose[0] = (*(s))[0];
      pose[1] = (*(s))[1];
      pose[2] = (*(s))[2];
      pose[3] = (*(s))[3];
      pose[4] = (*(s))[4];
      pose[5] = (*(s))[5];
      pose[6] = (*(s))[6];

      //printf("Pose in state validity checking is %f, %f, %f, %f, %f, %f, %f\n", pose[0], pose[1], pose[2], pose[3],
      //                                                                          pose[4], pose[5], pose[6]);

      //num_calls++;
      num[0]++;
      double dist;
      bool valid = cc_->isStateValid(pose, false, false, dist);
      if(valid){
        //num_valid++;
        num[3]++;
        ROS_DEBUG("state is valid");
      }
      else{
        //fail_collision++;
          num[2]++;
          ROS_DEBUG("state is not valid");
      }
      return valid;
    }

    void reset_count()
    {
      num[0] = 0;
      num[1] = 0;
      num[2] = 0;
      num[3] = 0;
    }

    void print_checks()
    {
      printf("num_calls=%d failed_ik=%d failed_collision=%d num_valid=%d\n",num[0],num[1],num[2],num[3]);
    }

  private:
    
    int *num;
    sbpl_arm_planner::CollisionChecker* cc_;
};


// class OMPLArmMotionValidator : public ompl::base::MotionValidator
// {
// public:
//     OMPLArmMotionValidator(const ompl::base::SpaceInformationPtr &si_) : ompl::base::MotionValidator(si_){}

//     void initialize(sbpl_arm_planner::CollisionChecker* c)
//     {
//       cc_ = c;
//       num = new int[4];
//     }

//     virtual bool checkMotion (const ompl::base::State *s1, const ompl::base::State *s2) const{
//       std::pair<ompl::base::State *, double> lastValid;
//       return checkMotion(s1, s2, lastValid);
//     }

//     virtual bool checkMotion (const ompl::base::State *s1, const ompl::base::State *s2, std::pair< ompl::base::State *, double > &lastValid) const{
//     double edge_length = si_->distance(s1, s2);
//     //printf("Checkmotion edge length is %f\n", edge_length);
//     int num_segments = static_cast<int>(edge_length / 0.001);
//     double interp_factor = 1.0 / static_cast<double>(num_segments);
//     bool valid = true;
//     ompl::base::State *interp_state = si_->allocState();
//     for (double t = 0 ; t <=1; t+=interp_factor) {
//         si_->getStateSpace()->interpolate(s1, s2, t, interp_state);
//         if (!check_isValid(interp_state)) {
//         valid = false;
//         break;
//         }
//     }
  
//     si_->freeState(interp_state);
  
//     return valid;
//     }

//     bool check_isValid(const ompl::base::State *state) const
//     {
//       const ompl::base::RealVectorStateSpace::StateType* s = dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*> (state);

//       std::vector<double> pose(7,0);
//       pose[0] = (*(s))[0];
//       pose[1] = (*(s))[1];
//       pose[2] = (*(s))[2];
//       pose[3] = (*(s))[3];
//       pose[4] = (*(s))[4];
//       pose[5] = (*(s))[5];
//       pose[6] = (*(s))[6];

//       //num_calls++;
//       num[0]++;
//       double dist;
//       bool valid = cc_->isStateValid(pose, false, false, dist);
//       if(valid){
//         //num_valid++;
//         num[3]++;
//         ROS_DEBUG("state is valid");
//       }
//       else{
//         //fail_collision++;
//           num[2]++;
//           ROS_DEBUG("state is not valid");
//       }
//       return valid;
//     }

//   private:
    
//     int *num;
//     sbpl_arm_planner::CollisionChecker* cc_;

// };


namespace sbpl_arm_planner {

class OMPLArmPlannerInterface
{
  public:

    OMPLArmPlannerInterface(sbpl_arm_planner::CollisionChecker* cc);
    ~OMPLArmPlannerInterface();

    bool init();

    ompl::base::SpaceInformationPtr getSPI();

    bool setStartAndGoal(const std::vector<double> &start_angles, const std::vector<double> &goal_angles);

    bool plan(double allowed_time, std::vector<trajectory_msgs::JointTrajectoryPoint> &ptraj, std::vector<trajectory_msgs::JointTrajectoryPoint> &straj, std::vector<double> &times);

    void setPlannerID(int p){ planner_id = p;};  
  private:
   
    int planner_id;
    ros::NodeHandle nh;
    ros::NodeHandle ph_;

    std::string ompl_planner_id_;
    std::string timeout_file_;

    ompl::base::StateSpacePtr armSpace;
    ompl::base::ProblemDefinition* pdef;
    ompl::base::Planner* planner;
    ompl::geometric::PathSimplifier* pathSimplifier;
    OMPLArmCollisionChecker* ompl_checker;
    sbpl_arm_planner::CollisionChecker* cc_;

    ompl::base::SpaceInformationPtr si_;
};
}
#endif
