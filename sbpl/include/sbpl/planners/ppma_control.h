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
#ifndef _PPMA_CONTROL_PLANNER_H_
#define _PPMA_CONTROL_PLANNER_H_

#include "../../sbpl/headers.h"
#include <sbpl/discrete_space_information/environment_ppma.h>

// often useful headers:
#include "/usr/local/include/ompl/util/RandomNumbers.h"
#include "/usr/local/include/ompl/tools/config/SelfConfig.h"
#include "/usr/local/include/ompl/datastructures/NearestNeighbors.h"
#include "/usr/local/include/ompl/control/planners/PlannerIncludes.h"
#include "/usr/local/include/ompl/base/goals/GoalSampleableRegion.h"

#include <queue>

class PPMAControlLazyListElement;

class PPMAControlState: public AbstractSearchState {
 public:
  int id;
  unsigned int v;
  unsigned int g;
  int h;
  short unsigned int iteration_closed;
  short unsigned int replan_number;
  PPMAControlState *best_parent;
  PPMAControlState *expanded_best_parent;
  bool in_incons;
  std::priority_queue<PPMAControlLazyListElement> lazyList;
  bool isTrueCost;
};


class PPMAControlLazyListElement {
 public:
  PPMAControlLazyListElement(PPMAControlState *p, int ec, bool itc) {
    parent = p;
    edgeCost = ec;
    isTrueCost = itc;
  }
  bool operator< (const PPMAControlLazyListElement &other) const {
    return (parent->v + edgeCost > other.parent->v + other.edgeCost);
  }
  PPMAControlState *parent;
  int edgeCost;
  bool isTrueCost;
};

class PPMAControlPlanner : public SBPLPlanner, public ompl::base::Planner {

 public:
  
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
                     ReplanParams params, int *solcost);
  virtual int replan(std::vector<int> *solution_stateIDs_V, ReplanParams params);
  virtual int replan(std::vector<int> *solution_stateIDs_V, ReplanParams params,
                     int *solcost);

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

  PPMAControlPlanner(const ompl::control::SpaceInformationPtr &si,
               EnvironmentPPMA *environment, bool bforwardsearch, double alloc_time, ReplanParams* get_params);
  ~PPMAControlPlanner();

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
  /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself */
  bool getIntermediateStates() const
  {
      return addIntermediateStates_;
  }

  /** \brief Specify whether the intermediate states generated along motions are to be added to the tree itself */
  void setIntermediateStates(bool addIntermediateStates)
  {
      addIntermediateStates_ = addIntermediateStates;
            }
  template<template<typename T> class NN>
  void setNearestNeighbors() {
    monolithic_tree_.reset(new NN<ControlMotion *>());
    lattice_tree_.reset(new NN<ControlMotion *>());
  }
  virtual void setup();


 protected:
  //data structures (open and incons lists)
  CHeap heap;
  std::vector<PPMAControlState *> incons;
  std::vector<PPMAControlState *> states;

  EnvironmentPPMA* env_;

  //params
  ReplanParams params;
  bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
  PPMAControlState *goal_state;
  PPMAControlState *start_state;
  int goal_state_id;
  int start_state_id;

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

  virtual PPMAControlState *GetState(int id);
  virtual void ExpandState(PPMAControlState *parent);
  virtual void EvaluateState(PPMAControlState *parent);
  void getNextLazyElement(PPMAControlState *state);
  void insertLazyList(PPMAControlState *state, PPMAControlState *parent, int edgeCost,
                      bool isTrueCost);
  void putStateInHeap(PPMAControlState *state);

  virtual int ImprovePath();

  virtual std::vector<int> GetSearchPath(int &solcost);

  virtual bool outOfTime();
  virtual void initializeSearch();
  virtual void prepareNextSearchIteration();
  virtual bool Search(std::vector<int> &pathIds, int &PathCost);

  //OMPL Stuff
  class ControlMotion {
   public:
    ControlMotion() : state(NULL), parent(NULL) {
    }
    ControlMotion(const ompl::control::SpaceInformation *si) : state(si->allocState()), control(si->allocControl()),
      parent(NULL), g(INFINITECOST), lattice_state(false), steps(0) {
    }
    ~ControlMotion() {
    }
    ompl::base::State *state;
    ControlMotion *parent;
    ompl::control::Control *control;
    unsigned int steps;
    int g;
    bool lattice_state;
  };
  void freeMemory();
  double distanceFunction(const ControlMotion *a, const ControlMotion *b) const {
    return si_->distance(a->state, b->state);
  }

  // Add the edge between state->parent to state to the monolithic tree. This
  // assumes that state->parent already exists in the tree.
  void AddToMonolithicTree(PPMAControlState * state);

  ompl::base::StateSamplerPtr sampler_;
  ompl::control::DirectedControlSamplerPtr control_sampler_;
  boost::shared_ptr<ompl::NearestNeighbors<ControlMotion*>> monolithic_tree_;
  boost::shared_ptr<ompl::NearestNeighbors<ControlMotion*>> lattice_tree_;
  double goalBias_;
  double maxDistance_;
  bool addIntermediateStates_;
  ompl::RNG rng_;
  ControlMotion *lastGoalMotion_;
  ompl::base::Goal *goal_;
  ompl::base::GoalSampleableRegion *goal_s_;

  // base::SpaceInformation cast as control::SpaceInformation for convenice.
  const ompl::control::SpaceInformation *si_c_;
};

#endif
