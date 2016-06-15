/*
 * Copyright (c) 2015, Venkatraman Narayanan, Sandip Aine and Maxim Likhachev
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
#ifndef _PPMA_PLANNER_H_
#define _PPMA_PLANNER_H_

#include "../../sbpl/headers.h"
#include <sbpl/discrete_space_information/environment_ppma.h>


#include "/usr/local/include/ompl/base/Planner.h"
// often useful headers:
#include "/usr/local/include/ompl/util/RandomNumbers.h"
#include "/usr/local/include/ompl/tools/config/SelfConfig.h"
#include "/usr/local/include/ompl/geometric/planners/PlannerIncludes.h"
#include "/usr/local/include/ompl/datastructures/NearestNeighbors.h"
#include "/usr/local/include/ompl/base/goals/GoalSampleableRegion.h"
#include "/usr/local/include/ompl/base/Cost.h"
#include "/usr/local/include/ompl/control/planners/rrt/RRT.h"
#include "/usr/local/include/ompl/geometric/PathGeometric.h"
#include "/usr/local/include/ompl/base/ProblemDefinition.h"

#include <queue>

class PPMALazyListElement;

enum class PlannerMode {
  H_STAR = 0,
  wA_STAR,
  RRT
};

class PPMAState: public AbstractSearchState {
 public:
  int id;
  unsigned int v;
  unsigned int g;
  int h;
  short unsigned int iteration_closed;
  short unsigned int replan_number;
  PPMAState *best_parent;
  PPMAState *expanded_best_parent;
  bool in_incons;
  std::priority_queue<PPMALazyListElement> lazyList;
  bool isTrueCost;
};


class PPMALazyListElement {
 public:
  PPMALazyListElement(PPMAState *p, int ec, bool itc) {
    parent = p;
    edgeCost = ec;
    isTrueCost = itc;
  }
  bool operator< (const PPMALazyListElement &other) const {
    return (parent->v + edgeCost > other.parent->v + other.edgeCost);
  }
  PPMAState *parent;
  int edgeCost;
  bool isTrueCost;
};

class PPMAPlanner : public SBPLPlanner, public ompl::base::Planner {

 public:
   PlannerMode planner_mode_;
  
  ReplanParams replan_params;
  std::vector<int> solution_stateIDs_V;
  
  virtual int replan(double allocated_time_secs,
                     std::vector<int> *solution_stateIDs_V) {
    printf("Not supported. Use ReplanParams");
    return -1;
  };
  virtual int replan(double allocated_time_sec,
                     std::vector<int> *solution_stateIDs_V, int *solcost) {
    printf("Not supported. Use ReplanParams");
    return -1;
  };

  virtual int replan(int start, int goal, std::vector<int> *solution_stateIDs_V,
                     ReplanParams params, int *solcost, double &totalT);
  virtual int replan(std::vector<int> *solution_stateIDs_V, ReplanParams params, double &totalT);
  virtual int replan(std::vector<int> *solution_stateIDs_V, ReplanParams params,
                     int *solcost, double &totalT);

  void interrupt();

  virtual int set_goal(int goal_stateID);
  virtual int set_start(int start_stateID);

  virtual void costs_changed(StateChangeQuery const &stateChange) {
    return;
  };
  virtual void costs_changed() {
    return;
  };

  virtual int force_planning_from_scratch() {
    return 1;
  };
  virtual int force_planning_from_scratch_and_free_memory() {
    return 1;
  };

  virtual int set_search_mode(bool bSearchUntilFirstSolution) {
    printf("Not supported. Use ReplanParams");
    return -1;
  };

  virtual void set_initialsolution_eps(double initialsolution_eps) {
    printf("Not supported. Use ReplanParams");
  };

  PPMAPlanner(const ompl::base::SpaceInformationPtr &si,
               EnvironmentPPMA *environment, bool bforwardsearch, double alloc_time, ReplanParams* get_params);
  ~PPMAPlanner();

  virtual void get_search_stats(std::vector<PlannerStats> *s);

  // OMPL stuff
  void InitializeOMPL();

  virtual void getPlannerData(ompl::base::PlannerData &data) const;
  virtual ompl::base::PlannerStatus solve(const
                                          ompl::base::PlannerTerminationCondition
                                          &ptc);
  virtual void clear();

  void setGoalBias(double goalBias) {
    goalBias_ = goalBias;
  }
  double getGoalBias() const {
    return goalBias_;
  }
  void setRange(double distance) {
    maxDistance_ = distance;
  }
  double getRange() const {
    return maxDistance_;
  }
  template<template<typename T> class NN>
  void setNearestNeighbors() {
    monolithic_tree_.reset(new NN<Motion *>());
  }
  virtual void setup();

  void setpdefstartgoal(std::vector<double> start, std::vector<double> goal);

  std::ofstream output;
  std::string results_file;


 protected:
  //data structures (open and incons lists)
  CHeap heap;
  std::vector<PPMAState *> incons;
  std::vector<PPMAState *> states;

  EnvironmentPPMA* env_;

  //params
  ReplanParams params;
  bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
  PPMAState *goal_state;
  PPMAState *start_state;
  int goal_state_id;
  int start_state_id;
  std::string sbpl_planner_id_;

  //search member variables
  double eps;
  double eps_satisfied;
  int search_expands;
  clock_t TimeStarted;
  short unsigned int search_iteration;
  short unsigned int replan_number;
  bool use_repair_time;

  //stats
  std::vector<PlannerStats> stats;
  unsigned int totalExpands;
  double totalTime;
  double totalPlanTime;
  double reconstructTime;

  bool interruptFlag;

  ros::NodeHandle ph_;

  virtual PPMAState *GetState(int id);
  virtual void ExpandState(PPMAState *parent);
  virtual void EvaluateState(PPMAState *parent);
  void getNextLazyElement(PPMAState *state);
  void insertLazyList(PPMAState *state, PPMAState *parent, int edgeCost,
                      bool isTrueCost);
  void putStateInHeap(PPMAState *state);

  virtual int ImprovePath();

  virtual std::vector<int> GetSearchPath(int &solcost);

  virtual bool outOfTime();
  virtual void initializeSearch();
  virtual void prepareNextSearchIteration();
  virtual bool Search(std::vector<int> &pathIds, int &PathCost);

  //OMPL Stuff
  class Motion {
   public:
    Motion() : state(NULL), parent(NULL) {
    }
    Motion(const ompl::base::SpaceInformationPtr &si) : state(si->allocState()),
      parent(NULL), g(INFINITECOST), lattice_state(false) {
    }
    ~Motion() {
    }
    ompl::base::State *state;
    Motion *parent;
    int g;
    bool lattice_state;
  };
  void freeMemory();
  double distanceFunction(const Motion *a, const Motion *b) const {
    return si_->distance(a->state, b->state);
  }

  // Add the edge between state->parent to state to the monolithic tree. This
  // assumes that state->parent already exists in the tree.
  void AddToMonolithicTree(PPMAState * state);

  ompl::base::StateSamplerPtr sampler_;
  boost::shared_ptr<ompl::NearestNeighbors<Motion*>> monolithic_tree_;
  double goalBias_;
  double maxDistance_;
  ompl::RNG rng_;
  Motion *lastGoalMotion_;
  ompl::base::Goal *goal_;
  ompl::base::GoalSampleableRegion *goal_s_;
  //ompl::base::ProblemDefinitionPtr pdef_;
};

#endif
