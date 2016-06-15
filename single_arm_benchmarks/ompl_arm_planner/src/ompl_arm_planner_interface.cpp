#include <ompl_arm_planner/ompl_arm_planner_interface.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <sys/time.h>

#include <chrono>

using namespace std;

namespace sbpl_arm_planner{

OMPLArmPlannerInterface::OMPLArmPlannerInterface(sbpl_arm_planner::CollisionChecker* cc) : cc_(cc), ph_("~")
{
  ph_.param<std::string>("ompl_planner_id",ompl_planner_id_, "");
  ph_.param<std::string>("timeout_file",timeout_file_, "20_timeout");
  if(!strcmp(ompl_planner_id_.c_str(), "RRTConnect"))
    planner_id = 0;
  if(!strcmp(ompl_planner_id_.c_str(), "PRM"))
    planner_id = 1;
  if(!strcmp(ompl_planner_id_.c_str(), "RRTstar") && !strcmp(timeout_file_.c_str(), "20_timeout"))
    planner_id = 2;
  if(!strcmp(ompl_planner_id_.c_str(), "RRTstar") && !strcmp(timeout_file_.c_str(), "H_timeout"))
    planner_id = 5;
  if(!strcmp(ompl_planner_id_.c_str(), "RRT"))
    planner_id = 3;
  if(!strcmp(ompl_planner_id_.c_str(), "BITstar") && !strcmp(timeout_file_.c_str(), "20_timeout"))
    planner_id = 4;
  if(!strcmp(ompl_planner_id_.c_str(), "BITstar") && !strcmp(timeout_file_.c_str(), "H_timeout"))
    planner_id = 6;
}

bool OMPLArmPlannerInterface::init()
{
  if(cc_ == NULL)
    return false;

  //create the StateSpace (defines the dimensions and their bounds)
  ompl::base::RealVectorStateSpace* r7 = new ompl::base::RealVectorStateSpace(7);
  
  r7->setName("jointspace");
  r7->setDimensionName(0,"shoulder_pan_joint");   // change names to joints - names are purely for me 
  r7->setDimensionName(1,"shoulder_lift_joint");
  r7->setDimensionName(2,"upper_arm_roll_joint");
  r7->setDimensionName(3,"elbow_flex_joint");
  r7->setDimensionName(4,"forearm_roll_joint");
  r7->setDimensionName(5,"wrist_flex_joint");
  r7->setDimensionName(6,"wrist_roll_right");

  ompl::base::RealVectorBounds bounds(7);   // set bounds here for each dumension
  bounds.setLow(0,-2.135398);
  bounds.setHigh(0,0.715);
  bounds.setLow(1,-0.355);
  bounds.setHigh(1,1.2963);
  bounds.setLow(2,-3.75);
  bounds.setHigh(2,0.865);
  bounds.setLow(3,-2.1213);
  bounds.setHigh(3,0.150);
  bounds.setLow(4,-M_PI);
  bounds.setHigh(4,M_PI);
  bounds.setLow(5,-2.0);
  bounds.setHigh(5,0.0);
  bounds.setLow(6,-M_PI);
  bounds.setHigh(6,M_PI);
  r7->setBounds(bounds);
  ompl::base::StateSpacePtr r7_p(r7);
  armSpace = r7_p;
  //armSpace->addSubSpace(r7_p, 1.0);

  //Define our SpaceInformation (combines the state space and collision checker)
  si_.reset(new ompl::base::SpaceInformation(armSpace));
  ompl_checker = new OMPLArmCollisionChecker(si_);
  ompl_checker->initialize(cc_);
  ompl::base::StateValidityChecker* temp2 = ompl_checker;
  si_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(temp2));
  si_->setStateValidityCheckingResolution(0.0348); // 0.1% this is the percentage of smallest joint range
  si_->setup();
  ROS_INFO("max extent: %f",si_->getMaximumExtent());

  //Define a ProblemDefinition (a start/goal pair)
  pdef = new ompl::base::ProblemDefinition(si_);

  //Create the planner
  if(planner_id==0)
    planner = new ompl::geometric::RRTConnect(si_);
  else if(planner_id==1)
    planner = new ompl::geometric::PRM(si_);
  else if(planner_id==2 || planner_id==5)
    planner = new ompl::geometric::RRTstar(si_);
  else if(planner_id==3)
    planner = new ompl::geometric::RRT(si_);
  else if(planner_id==4 || planner_id==6)
    planner = new ompl::geometric::BITstar(si_);
  else
    exit(1);
    
  planner->setProblemDefinition(ompl::base::ProblemDefinitionPtr(pdef));

  if(planner_id != 2 && planner_id != 5 && planner_id != 4 && planner_id != 6)
  {
    planner->setup();
    ROS_INFO("[ompl] Using planner: %d (%s)", planner_id, planner->getName().c_str());
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
  }

  pathSimplifier = new ompl::geometric::PathSimplifier(si_);
  
  return true;
}

ompl::base::SpaceInformationPtr OMPLArmPlannerInterface::getSPI()
{
  return si_;
}

bool OMPLArmPlannerInterface::setStartAndGoal(const std::vector<double> &start_angles, const std::vector<double> &goal_angles)
{
  // make start state
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> ompl_start(armSpace);
  (*(ompl_start))[0] = start_angles[0];
  (*(ompl_start))[1] = start_angles[1];
  (*(ompl_start))[2] = start_angles[2];
  (*(ompl_start))[3] = start_angles[3];
  (*(ompl_start))[4] = start_angles[4];
  (*(ompl_start))[5] = start_angles[5];
  (*(ompl_start))[6] = start_angles[6];

  /*
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> ompl_start(armSpace);
  (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[0] = start_angles[0];
  (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[1] = start_angles[1];
  (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[2] = start_angles[2];
  (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[3] = start_angles[3];
  (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[4] = start_angles[4];
  (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[5] = start_angles[5];
  (*(ompl_start->as<ompl::base::RealVectorStateSpace::StateType>(0)))[6] = start_angles[6];
  */

  if(planner->getSpaceInformation()->isValid(ompl_start.get()))
    ROS_INFO("[ompl] Start state is valid.");
  else
    ROS_ERROR("[ompl] Start state is NOT valid.");

  pdef->clearStartStates();
  ROS_INFO("[ompl] start_state: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
        (*(ompl_start))[0],
        (*(ompl_start))[1],
        (*(ompl_start))[2],
        (*(ompl_start))[3],
        (*(ompl_start))[4],
        (*(ompl_start))[5],
        (*(ompl_start))[6]);

  // make goal state
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> ompl_goal(armSpace);
  (*(ompl_goal))[0] = goal_angles[0];
  (*(ompl_goal))[1] = goal_angles[1];
  (*(ompl_goal))[2] = goal_angles[2];
  (*(ompl_goal))[3] = goal_angles[3];
  (*(ompl_goal))[4] = goal_angles[4];
  (*(ompl_goal))[5] = goal_angles[5];
  (*(ompl_goal))[6] = goal_angles[6];

  if(planner->getSpaceInformation()->isValid(ompl_goal.get()))
    ROS_INFO("[ompl] Goal state is valid.");
  else
    ROS_ERROR("[ompl] Goal state is NOT valid.");

  ROS_INFO("[ompl] goal_state: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
        (*(ompl_goal))[0],
        (*(ompl_goal))[1],
        (*(ompl_goal))[2],
        (*(ompl_goal))[3],
        (*(ompl_goal))[4],
        (*(ompl_goal))[5],
        (*(ompl_goal))[6]);

  pdef->setStartAndGoalStates(ompl_start,ompl_goal);

  //for RRT*
  ompl::base::GoalState* temp_goal = new ompl::base::GoalState(planner->getSpaceInformation());
  temp_goal->setState(ompl_goal);
  if(planner_id==2)
  {
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(armSpace));
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(obj->infiniteCost());

    planner->getProblemDefinition()->setOptimizationObjective(obj);
    planner->setup();
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
  }
  if(planner_id==5)
  {
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(armSpace));
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));

    planner->getProblemDefinition()->setOptimizationObjective(obj);
    planner->setup();
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
  }
  else if(planner_id==3)
  {
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(armSpace));
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    planner->getProblemDefinition()->setOptimizationObjective(obj);
  }

  if(planner_id == 4)
  {
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(armSpace));
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(obj->infiniteCost());

    planner->getProblemDefinition()->setOptimizationObjective(obj);
    planner->setup();
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
  }
  if(planner_id==6)
  {
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(armSpace));
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));

    planner->getProblemDefinition()->setOptimizationObjective(obj);
    planner->setup();
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
  }

  return true;
}


bool OMPLArmPlannerInterface::plan(double allowed_time, std::vector<trajectory_msgs::JointTrajectoryPoint> &ptraj, std::vector<trajectory_msgs::JointTrajectoryPoint> &straj, std::vector<double> &times)
{
  ROS_INFO("[ompl] planner: %d (%s)", planner_id, planner->getName().c_str());
  //planner->printProperties(std::cout);
  //planner->printSettings(std::cout);

  ptraj.clear();
  straj.clear();
  //time_t tt0, tt1, tt2, tt3;

  ompl_checker->reset_count();
  double t0 = ros::Time::now().toSec();
  clock_t ct0 = clock();
  chrono::time_point<chrono::monotonic_clock> start, end;
  start = chrono::monotonic_clock::now();
  //time(&tt0);
  //ros::Time rt0 = ros::Time::now();
  //ROS_INFO("[ompl] start clock (%lf)", t0);
  ompl::base::PlannerStatus res = planner->solve(allowed_time);
  //ros::Time rt1 = ros::Time::now();
  //time(&tt1);
  end = chrono::monotonic_clock::now();

  clock_t ct1 = clock();
  chrono::duration<double> elapsed_seconds = end-start;
  double t1 = ros::Time::now().toSec();
  //double rtplan = ros::Duration(rt1-rt0).toSec();

  //ROS_INFO("[ompl] end clock   (time_t: %f   clock_t: %f)", difftime(tt1, tt0), (ct1-ct0)/(double)CLOCKS_PER_SEC);
  //ROS_INFO("[ompl] end clock   (%lf) (%f)  (clock_t: %f)", t1, t1, (ct1-ct0)/(double)CLOCKS_PER_SEC);
  //ROS_INFO("[ompl] rostime planning time: %f", rtplan);
  ROS_INFO("[ompl] planner result: %s", res.asString().c_str());

  times.clear(); 
  times.resize(2,-100000);
  //times[0] = (ct1-ct0)/(double)CLOCKS_PER_SEC; //difftime(tt1, tt0); //(ct1-ct0)/(double)CLOCKS_PER_SEC; 
  printf("[clock] Clocks per sec %f\n", (double)CLOCKS_PER_SEC);
  //times[0] = t1 - t0;
  times[0] = elapsed_seconds.count();
  if(res.asString().compare("Exact solution") != 0)
  {
    ROS_ERROR("[ompl] Planner failed to find exact solution in %0.8fsec.", times[0]);
    planner->getProblemDefinition()->clearSolutionPaths();
    planner->clear();
    return false;
  }
  ompl_checker->print_checks();

  ompl::base::PathPtr path = planner->getProblemDefinition()->getSolutionPath();

  // planning failed
  if(!path)
  {
    ROS_ERROR("[ompl] Failed to plan.");
    planner->getProblemDefinition()->clearSolutionPaths();
    planner->clear();
    return false;
  }

  ompl::geometric::PathGeometric geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);

  ptraj.resize(geo_path.getStateCount());
  for(unsigned int i = 0; i< geo_path.getStateCount(); i++)
  {
    ompl::base::State* state = geo_path.getState(i);
    //const ompl::base::RealVectorStateSpace::StateType* s = dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*> (state);
    const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
    ptraj[i].positions.resize(7);
    for(size_t j = 0; j < 7; ++j)
      ptraj[i].positions[j] = s->values[j];
  }
  // simplify & interpolate 
  ompl::geometric::PathGeometric geo_path2 = geo_path;
  //time(&tt2);
  double t2 = ros::Time::now().toSec();
  clock_t ct2 = clock();
  bool b1 = pathSimplifier->reduceVertices(geo_path2);
  bool b2 = pathSimplifier->collapseCloseVertices(geo_path2);
  clock_t ct3 = clock();
  double t3 = ros::Time::now().toSec();
  //time(&tt3);
  ROS_INFO("[ompl] reduce: %d     collapse:%d", b1, b2);
  times[1] = t3-t2; 
  //times[1] = (ct3-ct2)/(double)CLOCKS_PER_SEC; //difftime(tt3, tt2);
   
  // interpolating the simplified path
  // geo_path2.interpolate();
 
  straj.resize(geo_path2.getStateCount());
  for(unsigned int i = 0; i< geo_path2.getStateCount(); i++)
  {
    ompl::base::State* state = geo_path2.getState(i);
    //const ompl::base::RealVectorStateSpace::StateType* s = dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*> (state);
    const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
    straj[i].positions.resize(7);
    for(size_t j = 0; j < 7; ++j)
      straj[i].positions[j] = s->values[j];
  }

  planner->getProblemDefinition()->clearSolutionPaths();
  planner->clear();

  ROS_INFO("[ompl] planning time: %0.8fsec    processing time: %0.8fsec", times[0], times[1]);
  ROS_INFO("[ompl] clock_t planning time: %0.8fsec   clock_t processing time: %0.8fsec", double(ct1-ct0)/double(CLOCKS_PER_SEC), float(ct3-ct2)/CLOCKS_PER_SEC);
  std::cout << "CLOCK_T:   " << ct1 << "   "  << ct0; 
  ROS_INFO("[ompl] planned_path: %d   processed_path: %d", int(ptraj.size()), int(straj.size()));
  return true;
}

}
