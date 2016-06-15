/*
 * Copyright (c) 2013, Mike Phillips and Maxim Likhachev
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
#include <egraphs/egraph_planner.h>
#include <algorithm>
#include <numeric>

using namespace std;

template <typename HeuristicType>
MHEGraphPlanner<HeuristicType>::MHEGraphPlanner(DiscreteSpaceInformation* environment, int num_heurs,
                               EGraphManagerPtr egraph_mgr) :
  params(0.0), egraph_mgr_(egraph_mgr){ //, goal_state(NULL) {
  //bforwardsearch = bSearchForward;
  //if(!bSearchForward)
    //ROS_WARN("backward search not supported. setting to run forward search.");
  bforwardsearch = true;
  environment_ = environment;
  replan_number = 0;

  //goal_state_id = -1;
  start_state_id = -1;
  evaluated_snaps = 0;

  num_heuristics = num_heurs;
  heaps.resize(num_heuristics);
  incons.resize(num_heuristics);
  states.resize(num_heuristics);
  egraph_mgr_->setNumHeuristics(num_heurs);

  //Meta A*
  queue_expands.resize(num_heuristics, 0);

  // DTS
  queue_best_h.resize(num_heuristics, INFINITECOST);
  meta_queue_best_h.resize(num_heuristics, INFINITECOST);
  alpha.resize(num_heuristics, 1.0);
  beta.resize(num_heuristics, 1.0);
  betaC = 5;
  gsl_rng_env_setup();
  gsl_rand_T = gsl_rng_default;
  gsl_rand = gsl_rng_alloc(gsl_rand_T);

  //create heuristic threads
  num_threads_ = 0;
  sleeping_ = 0;
  reported_for_duty_ = 0;
  planner_ok_ = true;
  //threads_.resize(num_threads_);
  //for(int i=0; i<num_threads_; i++)
    //threads_[i] = new boost::thread(boost::bind(&MHEGraphPlanner<HeuristicType>::initializeHeuristics, this));
}

template <typename HeuristicType>
MHEGraphPlanner<HeuristicType>::~MHEGraphPlanner(){
  gsl_rng_free(gsl_rand);
  planner_ok_ = false;
  heuristic_cond_.notify_all();
};

template <typename HeuristicType>
LazyAEGState* MHEGraphPlanner<HeuristicType>::GetState(int q_id, int id){	
  //if this stateID is out of bounds of our state vector then grow the list
  if(id >= int(states[q_id].size())){
    for(int i=states[q_id].size(); i<=id; i++)
      states[q_id].push_back(NULL);
  }
  //if we have never seen this state then create one
  if(states[q_id][id]==NULL){
    states[q_id][id] = new LazyAEGState();
    states[q_id][id]->id = id;
    states[q_id][id]->replan_number = -1;
  }
  //initialize the state if it hasn't been for this call to replan
  LazyAEGState* s = states[q_id][id];
  if(s->replan_number != replan_number){
    s->g = INFINITECOST;
    s->v = INFINITECOST;
    s->iteration_closed = -1;
    s->replan_number = replan_number;
    s->best_parent = NULL;
    s->expanded_best_parent = NULL;
    s->best_edge_type = EdgeType::NONE;
    s->expanded_best_edge_type = EdgeType::NONE;
    s->snap_midpoint = -1;
    s->expanded_snap_midpoint = -1;
    s->heapindex = 0;
    s->in_incons = false;
    s->isTrueCost = true;
    //clear the lazy list
    while(!s->lazyList.empty())
      s->lazyList.pop();

    if(planner_type == PlannerType::IMHA){
      //compute heuristics
      if(bforwardsearch){
        //printf("getHeuristic q=%d id=%d\n",q_id,s->id);
        clock_t h_t0 = clock();
        s->h = egraph_mgr_->getHeuristic(q_id,s->id);
        clock_t h_t1 = clock();
        heuristicClock += h_t1-h_t0;
      } else {
        printf("backwards search not implemented!");
        assert(false);
        //s->h = environment_->GetStartHeuristic(s->id);
      }
    }
  }
  return s;
}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::ExpandState(int q_id, LazyAEGState* parent){
  bool print = false; //parent->id == 285566;
  if(print)
    printf("expand %d\n",parent->id);
  vector<int> children;
  vector<int> costs;
  vector<bool> isTrueCost;

  double getSucc_t0 = ros::Time::now().toSec();
  if(bforwardsearch)
    environment_->GetLazySuccsWithUniqueIds(parent->id, &children, &costs, &isTrueCost);
  else
    environment_->GetLazyPredsWithUniqueIds(parent->id, &children, &costs, &isTrueCost);
  double getSucc_t1 = ros::Time::now().toSec();
  succsClock += getSucc_t1-getSucc_t0;

  vector<EdgeType> edgeTypes(children.size(),EdgeType::NORMAL);
  
  vector<int> snap_midpoints(children.size(),-1);
  if (params.use_egraph){
    //egraph_mgr_->clearSnapSuccessorsCache();

    //egraph_mgr_->getSnapSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);
    double shortcut_t0 = ros::Time::now().toSec();
    egraph_mgr_->getDirectShortcutSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);

    snap_midpoints.resize(children.size(),-1);
    egraph_mgr_->getSnapShortcuts(parent->id, &children, &costs, &isTrueCost, &edgeTypes, &snap_midpoints);
    if(edgeTypes.size()>0 && edgeTypes.back()==EdgeType::SNAP_DIRECT_SHORTCUT)
      assert(snap_midpoints.back()>=0);
    double shortcut_t1 = ros::Time::now().toSec();
    shortcutClock += shortcut_t1 - shortcut_t0;

    //egraph_mgr_->getComboSnapShortcutSuccessors(parent->id, &children, &costs, &isTrueCost, &edgeTypes);
    // getComboSnapShortcutSuccessors needs the output of getSnapSuccessors, so
    // getSnapSuccessors caches its output inside egraph_mgr_. we must make sure
    // to clean it up before the next iteration
    //egraph_mgr_->clearSnapSuccessorsCache();
  }
  
  // make sure our costs are nonzero
  for (auto& cost : costs){
      assert(cost > 0);
  }
  if (print){
      for (auto& id : children){
          ROS_INFO("%d", id);
      }
  }

  //iterate through children of the parent

  if(planner_type == PlannerType::IMHA){
    for(int i=0; i<(int)children.size(); i++){
      //printf("  succ %d\n",children[i]);
      LazyAEGState* child = GetState(q_id,children[i]);
      insertLazyList(q_id, child, parent, costs[i], isTrueCost[i], edgeTypes[i], snap_midpoints[i]);
      if(child->h < meta_queue_best_h[q_id])
        meta_queue_best_h[q_id] = child->h;
    } 
  }
  else{
    for(int i=0; i<(int)children.size(); i++){
      //printf("  succ %d\n",children[i]);
      if(children[i] >= int(states[0].size()) || states[0][children[i]]==NULL){
        for(int j=0; j<num_heuristics; ++j) 
          GetState(j, children[i]);
        getAllHeuristics(children[i]);
      }
      if(states[0][children[i]]->replan_number != replan_number)
        getAllHeuristics(children[i]);
      for(int j=0; j<num_heuristics; ++j){
        LazyAEGState* child = GetState(j, children[i]);
        insertLazyList(j, child, parent, costs[i], isTrueCost[i], edgeTypes[i], snap_midpoints[i]);
        if(child->h < meta_queue_best_h[j])
          meta_queue_best_h[j] = child->h;
      }
    }
  }

  //Meta A*
  queue_expands[q_id]++;
}

//assumptions:
//state is at the front of the open list
//it's minimum f-value is an underestimate (the edge cost from the parent is a guess and needs to be evaluated properly)
//it hasn't been expanded yet this iteration
template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::EvaluateState(int q_id, LazyAEGState* state){
  bool print = false; //state->id == 285566;
  if(print)
    printf("evaluate %d (from %d)\n",state->id, state->best_parent->id);
  LazyAEGState* parent = state->best_parent;
  EdgeType edgeType = state->best_edge_type;
  int snap_midpoint = state->snap_midpoint;

  //printf("state_ptr=%p\n",state);
  //printf("state_id=%d\n",state->id);
  //printf("parent_ptr=%p\n",parent);
  //printf("parent_id=%d\n",parent->id);
  
  /* Mike replacing victor's code
  Edge snap_edge(parent->id, state->id);
  int trueCost;
  if (egraph_mgr_->snaps_.find(snap_edge) != egraph_mgr_->snaps_.end()){
    trueCost = egraph_mgr_->getSnapTrueCost(parent->id, state->id);
  } else {
    trueCost = environment_->GetTrueCost(parent->id, state->id);
  }
  */

  int trueCost;
  if(edgeType == EdgeType::SNAP)
    trueCost = egraph_mgr_->getSnapTrueCost(parent->id, state->id);
  else if(edgeType == EdgeType::NORMAL){
    double getSucc_t0 = ros::Time::now().toSec();
    trueCost = environment_->GetTrueCost(parent->id, state->id);
    double getSucc_t1 = ros::Time::now().toSec();
    succsClock += getSucc_t1-getSucc_t0;
  }
  else if(edgeType == EdgeType::SNAP_DIRECT_SHORTCUT){
    double shortcut_t0 = ros::Time::now().toSec();
    assert(snap_midpoint >= 0);
    trueCost = egraph_mgr_->getSnapShortcutTrueCost(parent->id, snap_midpoint, state->id);
    double shortcut_t1 = ros::Time::now().toSec();
    shortcutClock += shortcut_t1 - shortcut_t0;
  }
  else
    assert(false);

  //DTS
  int prev_best_h = queue_best_h[q_id];

  if(planner_type == PlannerType::IMHA){
    getNextLazyElement(q_id, state);
    if(trueCost > 0){ //if the evaluated true cost is valid (positive), insert it into the   lazy list
      insertLazyList(q_id, state,parent,trueCost,true,edgeType,snap_midpoint);
      //DTS
      if(state->h < queue_best_h[q_id]){
        queue_best_h[q_id] = state->h;
        assert(queue_best_h[q_id] >= 0);
      }
    }
  }
  else{
    for(int j=0; j<num_heuristics; ++j){
      LazyAEGState* tmp_state = GetState(j, state->id);
      getNextLazyElement(j, tmp_state);
      if(trueCost > 0){ //if the evaluated true cost is valid (positive), insert it into the lazy list
        insertLazyList(j, tmp_state,parent,trueCost,true,edgeType,snap_midpoint);
        //DTS
        if(tmp_state->h < queue_best_h[j]){
          queue_best_h[j] = tmp_state->h;
          if(queue_best_h[j] == 0){
            //ROS_ERROR("queue %d is done",j);
            //std::cin.get();
          }
          assert(queue_best_h[j] >= 0);
        }
      }
    }
  }

  // DTS
  double reward = 0;
  if(queue_best_h[q_id] < prev_best_h){
    reward = 1;
  }
  else{
    //ROS_ERROR("queue %d got no reward!",q_id);
    //std::cin.get();
  }
  if(alpha[q_id] + beta[q_id] < betaC){
    alpha[q_id] = alpha[q_id] + reward;
    beta[q_id] = beta[q_id] + (1-reward);
  }
  else{
    alpha[q_id] = (alpha[q_id] + reward)*betaC/(betaC+1);
    beta[q_id] = (beta[q_id] + (1-reward))*betaC/(betaC+1);
  }

  //TEMPORARY CODE TO FIND THE BIGGEST HEURISTIC DROP
  if(trueCost > 0){
    if (planner_type == PlannerType::IMHA){
      LazyAEGState* p = GetState(q_id, parent->id);
      LazyAEGState* child = GetState(q_id, state->id);
      int dec = p->h - child->h;
      if(dec > max_heur_dec){
        //max_heur_dec = dec;
        printf("a bigger h decrease %d for queue %d (parent=%d child=%d)\n",dec,q_id,p->h,     child->h);
        //std::cin.get();
      }
    }
    else{
      for (int j=0; j<num_heuristics; ++j){
        LazyAEGState* p = GetState(j, parent->id);
        LazyAEGState* child = GetState(j, state->id);
        int dec = p->h - child->h;
        if(dec > max_heur_dec){
          //max_heur_dec = dec;
          printf("a bigger h decrease %d for queue %d (parent=%d child=%d)\n",dec,j,p->h,child->h);
          //std::cin.get();
        }
      }
    }
  }
  //TEMPORARY CODE TO FIND THE BIGGEST HEURISTIC DROP

}

//this should only be used with EvaluateState since it is assuming state hasn't been expanded yet (only evaluated)
template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::getNextLazyElement(int q_id, LazyAEGState* state){
  if(state->lazyList.empty()){
    state->g = INFINITECOST;
    state->best_parent = NULL;
    state->best_edge_type = EdgeType::NONE;
    state->snap_midpoint = -1;
    state->isTrueCost = true;
    return;
  }
  LazyAEGListElement elem = state->lazyList.top();
  state->lazyList.pop();
  state->g = elem.parent->v + elem.edgeCost;
  state->best_parent = elem.parent;
  state->best_edge_type = elem.edgeType;
  state->snap_midpoint = elem.snap_midpoint;
  state->isTrueCost = elem.isTrueCost;
  //the new value is cheapest and if the value is also true then we want to throw out all the other options
  if(state->isTrueCost){
    while(!state->lazyList.empty())
      state->lazyList.pop();
  }
  putStateInHeap(q_id, state);
}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::insertLazyList(int q_id, LazyAEGState* state, LazyAEGState* parent, int edgeCost, bool isTrueCost, EdgeType edgeType, int snap_midpoint){
  bool print = false; //state->id == 285566 || parent->id == 285566;
  if(state->v <= parent->v + edgeCost)
    return;
  else if(state->g <= parent->v + edgeCost){
    //if the best g-value we have is true and better, then the value we had dominates this one and we don't need it
    if(state->isTrueCost)
      return;
    //insert this guy into the lazy list
    LazyAEGListElement elem(parent,edgeCost,isTrueCost,edgeType,snap_midpoint);
    state->lazyList.push(elem);
  }
  else{//the new guy is the cheapest so far
    //should we save what was the previous best?
    if(!isTrueCost && //the better guy's cost is not for sure
       //state->g < INFINITECOST && //we actually have a previous best (we actually don't need this line because of the next one)
       state->g < state->v){ //we're not saving something we already expanded (and is stored in v and expanded_best_parent)
      //we save it by putting it in the lazy list
      LazyAEGListElement elem(state->best_parent, state->g - state->best_parent->v, state->isTrueCost, state->best_edge_type, state->snap_midpoint);
      state->lazyList.push(elem);
      //printf("save the previous best\n");
    }

    //the new guy is the cheapest
    state->g = parent->v + edgeCost;
    state->best_parent = parent;
    state->best_edge_type = edgeType;
    state->snap_midpoint = snap_midpoint;
    state->isTrueCost = isTrueCost;

    //the new value is cheapest and if the value is also true then we want to throw out all the other options
    if(isTrueCost){
      //printf("clear the lazy list\n");
      while(!state->lazyList.empty())
        state->lazyList.pop();
    }

    //this function puts the state into the heap (or updates the position) if we haven't expanded
    //if we have expanded, it will put the state in the incons list (if we haven't already)
    putStateInHeap(q_id,state);
  }
  if(print)
    printf("state->id=%d state->g=%d state->h=%d, parent->v=%d edgeCost=%d isTrueCost=%d\n",state->id,state->g,state->h,parent->v,edgeCost,isTrueCost);
}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::putStateInHeap(int q_id, LazyAEGState* state){
  if(updateGoal(state))
    return;
  bool print = false; //state->id == 285566;
  //we only allow one expansion per search iteration
  //so insert into heap if not closed yet
  if(state->iteration_closed != search_iteration){
    CKey key;
    key.key[0] = state->g + int(eps * state->h);
    if(print)
      printf("put state in open with f %lu\n", key.key[0]);
    //if the state is already in the heap, just update its priority
    if(state->heapindex != 0)
      heaps[q_id].updateheap(state,key);
    else //otherwise add it to the heap
      heaps[q_id].insertheap(state,key);
  }
  //if the state has already been expanded once for this iteration
  //then add it to the incons list so we can keep track of states
  //that we know we have better costs for
  else if(!state->in_incons){
    if(print)
      printf("put state in incons\n");
    incons[q_id].push_back(state);
    state->in_incons = true;
  }
}

template <typename HeuristicType>
bool MHEGraphPlanner<HeuristicType>::updateGoal(LazyAEGState* state){
  if(egraph_mgr_->egraph_env_->isGoal(state->id) && state->isTrueCost && state->g < goal_state.g){
    //ROS_INFO("updating the goal state");
    goal_state.id = state->id;
    goal_state.g = state->g;
    goal_state.best_parent = state->best_parent;
    goal_state.best_edge_type = state->best_edge_type;
    goal_state.snap_midpoint = state->snap_midpoint;
    goal_state.isTrueCost = true;
  }
  return false;
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
template <typename HeuristicType>
int MHEGraphPlanner<HeuristicType>::ImprovePath(){

  //expand states until done
  int expands = 0;
  bool spin_again = false;
  int q_id = 0;
  CKey min_key = heaps[0].getminkeyheap();
  while(!heaps[0].emptyheap() && 
        min_key.key[0] < INFINITECOST && 
        (goal_state.g > min_key.key[0] || !goal_state.isTrueCost) &&
        (goal_state.v > min_key.key[0]) &&
        !outOfTime()){

    if(!spin_again || meta_search_type != MetaSearchType::DTS){
      double meta_t0 = ros::Time::now().toSec();
      q_id = GetBestHeuristicID();
      double meta_t1 = ros::Time::now().toSec();
      metaTime += meta_t1-meta_t0;
    }
    else{
      //printf("spin again %d\n",q_id);
    }
    printf("q_id=%d\n",q_id);

    //checkHeaps("before delete heap");
    
    //get the state		
    LazyAEGState* state = (LazyAEGState*)heaps[q_id].deleteminheap();
    // delete from the other queues as well if SMHA.
    if (planner_type == PlannerType::SMHA){
      for (int j = 0; j < num_heuristics; ++j) {
        if (j != q_id){
          LazyAEGState* state_to_delete = GetState(j, state->id);
          heaps[j].deleteheap(state_to_delete);
        }
      }
    }

    //checkHeaps("after delete heap");

    if(state->v == state->g){
      printf("ERROR: consistent state is being expanded\n");
      printf("id=%d v=%d g=%d isTrueCost=%d lazyListSize=%lu\n",
              state->id,state->v,state->g,state->isTrueCost,state->lazyList.size());
      //std::cin.get();
    }

    if(state->isTrueCost){
      //mark the state as expanded
      if (planner_type == PlannerType::IMHA){
        state->v = state->g;
        state->expanded_best_parent = state->best_parent;
        state->expanded_best_edge_type = state->best_edge_type;
        state->expanded_snap_midpoint = state->snap_midpoint;
        state->iteration_closed = search_iteration;
      }
      else{
        for(int j=0; j<num_heuristics; j++){
          LazyAEGState* tmp_state = GetState(j, state->id);
          tmp_state->v = tmp_state->g;
          tmp_state->expanded_best_parent = tmp_state->best_parent;
          tmp_state->expanded_best_edge_type = tmp_state->best_edge_type;
          tmp_state->expanded_snap_midpoint = tmp_state->snap_midpoint;
          tmp_state->iteration_closed = search_iteration;
        }
      }
      //expand the state
      expands++;
      ExpandState(q_id, state);
      spin_again = true;
      if(expands%10000 == 0)
        printf("expands so far=%u\n", expands);
    }
    else{ //otherwise the state needs to be evaluated for its true cost
      EvaluateState(q_id, state);
      spin_again = false;
    }

    //get the min key for the next iteration
    min_key = heaps[0].getminkeyheap();
  }

  search_expands += expands;

  if(goal_state.v < goal_state.g){
    goal_state.g = goal_state.v;
    goal_state.best_parent = goal_state.expanded_best_parent;
    goal_state.best_edge_type = goal_state.expanded_best_edge_type;
    goal_state.snap_midpoint = goal_state.expanded_snap_midpoint;
  }

  if(goal_state.g == INFINITECOST && (heaps[0].emptyheap() || min_key.key[0] >= INFINITECOST))
    return 0;//solution does not exists
  if(!heaps[0].emptyheap() && goal_state.g > min_key.key[0])
    return 2; //search exited because it ran out of time
  printf("search exited with a solution for eps=%.2f\n", eps*params.epsE);
  if(goal_state.g < goal_state.v){
    goal_state.expanded_best_parent = goal_state.best_parent;
    goal_state.expanded_best_edge_type = goal_state.best_edge_type;
    goal_state.expanded_snap_midpoint = goal_state.snap_midpoint;
    goal_state.v = goal_state.g;
  }
  return 1;
}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::checkHeaps(string msg){
  if(planner_type == PlannerType::IMHA)
    return;

  bool sameSizes = true;
  for(int i=1; i<num_heuristics; i++){
    if(heaps[0].currentsize != heaps[i].currentsize){
      sameSizes = false;
      ROS_ERROR("%s",msg.c_str());
      ROS_ERROR("heap[0] has size %d and heap[%d] has size %d",heaps[0].currentsize,i,heaps[i].currentsize);
      std::cin.get();
    }
  }
  if(sameSizes)
    return;

  for(int i=1; i<=heaps[0].currentsize; ++i){
    LazyAEGState* state = (LazyAEGState*)heaps[0].heap[i].heapstate;
    for(int j=1; j<num_heuristics; j++){
      bool found = false;
      for(int k=1; k<=heaps[j].currentsize; ++k){
        LazyAEGState* state2 = (LazyAEGState*)heaps[j].heap[k].heapstate;
        if(state->id == state2->id){
          found = true;
          if(state->g != state2->g || 
             state->v != state2->v || 
             state->best_parent != state2->best_parent || 
             state->expanded_best_parent != state2->expanded_best_parent || 
             state->isTrueCost != state2->isTrueCost){
            ROS_ERROR("%s",msg.c_str());
            ROS_ERROR("state %d found in queues 0 and %d but state internals didn't match",state->id,j);
            ROS_ERROR("heap 0: g=%d v=%d parent=%p expanded_parent=%p trueCost=%d",state->g,state->v,state->best_parent,state->expanded_best_parent,state->isTrueCost);
            ROS_ERROR("heap %d: g=%d v=%d parent=%p expanded_parent=%p trueCost=%d",j,state2->g,state2->v,state2->best_parent,state2->expanded_best_parent,state2->isTrueCost);
            std::cin.get();
          }
          break;
        }
      }
      if(!found){
        ROS_ERROR("%s",msg.c_str());
        ROS_ERROR("heap[0] has state %d and heap[%d] doesn't",state->id,j);
        std::cin.get();
      }
    }
  }

}

template <typename HeuristicType>
bool MHEGraphPlanner<HeuristicType>::reconstructSuccs(LazyAEGState* state, 
                                               LazyAEGState*& next_state,
                                               vector<int>* wholePathIds,
                                               vector<int>* costs){
    //ROS_INFO("reconstruct with standard edge start");
    vector<int> SuccIDV;
    vector<int> CostV;
    vector<bool> isTrueCost;
    if(bforwardsearch)
        environment_->GetLazySuccsWithUniqueIds(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost);
    else
        environment_->GetLazyPredsWithUniqueIds(state->expanded_best_parent->id, &SuccIDV, &CostV, &isTrueCost);
    int actioncost = INFINITECOST;
    //ROS_INFO("reconstruct with standard edge %d\n",state->expanded_best_parent->id);
    for(unsigned int i=0; i<SuccIDV.size(); i++){
        //printf("  succ %d\n",SuccIDV[i]);
        if(SuccIDV[i] == state->id && CostV[i]<actioncost)
            actioncost = CostV[i];
    }
    if(actioncost == INFINITECOST){
        return false;
    } else {
        // remember we're starting from the goal and working backwards, so we
        // want to store the parents
        //ROS_INFO("good...");
        costs->push_back(actioncost);
        next_state = state->expanded_best_parent;
        wholePathIds->push_back(next_state->id);
        return true;
    }
}

template <typename HeuristicType>
vector<int> MHEGraphPlanner<HeuristicType>::GetSearchPath(int& solcost){
    double reconstruct_t0 = ros::Time::now().toSec();

    bool print = false;
    vector<int> wholePathIds;
    vector<int> costs;
    LazyAEGState* state;
    LazyAEGState* final_state;
    if(bforwardsearch){
        state = &goal_state;
        final_state = start_state;
    } else {
        state = start_state;
        final_state = &goal_state;
    } 

    //if the goal was not expanded but was generated (we don't have a bound on the solution)
    //pretend that it was expanded for path reconstruction and revert the state afterward
    bool goal_expanded = true;
    if(goal_state.expanded_best_parent==NULL){
      goal_expanded = false;
      goal_state.expanded_best_parent = goal_state.best_parent;
      goal_state.v = goal_state.g;
      goal_state.expanded_best_edge_type = goal_state.best_edge_type;
      goal_state.expanded_snap_midpoint = goal_state.snap_midpoint;
    }

    wholePathIds.push_back(state->id);
    solcost = 0;
    int shortcut_count = 0;

    while(state->id != final_state->id){
        if(state->expanded_best_parent == NULL){
            printf("a state along the path has no parent!\n");
            assert(false);
        }
        if(state->v == INFINITECOST){
            printf("a state along the path has an infinite g-value!\n");
            printf("inf state = %d\n",state->id);
            assert(false);
        }

        LazyAEGState* next_state;
        if(state->expanded_best_edge_type == EdgeType::SNAP){
          assert(egraph_mgr_->reconstructSnap(state, next_state, &wholePathIds, &costs));
          if(print)
            ROS_INFO("snap edge %d %d %d", costs.back(), state->id, wholePathIds.back());
        }
        else if(state->expanded_best_edge_type == EdgeType::NORMAL){
          bool ret = reconstructSuccs(state, next_state, &wholePathIds, &costs);
          assert(ret);
          if(print)
            ROS_INFO("normal edge %d %d %d", costs.back(), state->id, wholePathIds.back());
        }
        else if(state->expanded_best_edge_type == EdgeType::DIRECT_SHORTCUT){
          int sc_cost;
          assert(egraph_mgr_->reconstructDirectShortcuts(state, next_state, &wholePathIds, &costs, shortcut_count, sc_cost));
          if(print)
            ROS_INFO("shortcut edge %d %d %d", sc_cost, state->id, wholePathIds.back());
        }
        else if(state->expanded_best_edge_type == EdgeType::SNAP_DIRECT_SHORTCUT){
          int totalCost;
          assert(egraph_mgr_->reconstructSnapShortcut(state, next_state, &wholePathIds, &costs, totalCost));
          if(print)
            ROS_INFO("snap shortcut edge %d %d %d", totalCost, state->id, wholePathIds.back());
        }
        else
          assert(false);
        assert(next_state == state->expanded_best_parent);
        assert(wholePathIds.back() == state->expanded_best_parent->id);
        state = next_state;
    }

    //if we pretended that the goal was expanded for path reconstruction then revert the state now
    if(!goal_expanded){
      goal_state.expanded_best_parent = NULL;
      goal_state.v = INFINITECOST;
    }

    //if we searched forward then the path reconstruction 
    //worked backward from the goal, so we have to reverse the path
    if(bforwardsearch){
        std::reverse(wholePathIds.begin(), wholePathIds.end());
        std::reverse(costs.begin(), costs.end());
    }
    solcost = std::accumulate(costs.begin(), costs.end(), 0);

    double reconstruct_t1 = ros::Time::now().toSec();
    reconstructTime = reconstruct_t1 - reconstruct_t0;

    /*
    // if we're using lazy evaluation, we always want to feedback the path
    // regardless if it's valid
    if (params.feedback_path || params.use_lazy_validation){
        egraph_mgr_->storeLastPath(wholePathIds, costs);
    }
    if (params.use_lazy_validation){
        egraph_mgr_->feedbackLastPath();
    }
    */

    double feedback_t0 = ros::Time::now().toSec();
    if (params.feedback_path){
        egraph_mgr_->storeLastPath(wholePathIds, costs);
        egraph_mgr_->feedbackLastPath();
    }
    double feedback_t1 = ros::Time::now().toSec();
    feedbackPathTime = feedback_t1-feedback_t0;
    //egraph_mgr_->printVector(wholePathIds);
    return wholePathIds;
}

template <typename HeuristicType>
bool MHEGraphPlanner<HeuristicType>::outOfTime(){
  //if the user has sent an interrupt signal we stop
  if(interruptFlag)
    return true;
  //if we are supposed to run until the first solution, then we are never out of time
  if(params.return_first_solution)
    return false;
  double time_used = ros::Time::now().toSec() - TimeStarted;
  if(time_used >= params.max_time)
    printf("out of max time\n");
  if(use_repair_time && eps_satisfied != INFINITECOST && time_used >= params.repair_time)
    printf("used all repair time...\n");
  //we are out of time if:
         //we used up the max time limit OR
         //we found some solution and used up the minimum time limit
  return time_used >= params.max_time || 
         (use_repair_time && eps_satisfied != INFINITECOST && time_used >= params.repair_time);
}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::initializeSearch(){

  //it's a new search, so increment replan_number and reset the search_iteration
  replan_number++;
  search_iteration = 0;
  search_expands = 0;
  totalExpands = 0;
  succsClock = 0;
  shortcutClock = 0;
  heuristicClock = 0;
  reconstructTime = 0;
  feedbackPathTime = 0;
  metaTime = 0;

  //clear open list, incons list, and stats list
  for(int i=0; i<num_heuristics; i++){
    heaps[i].makeemptyheap();
    incons[i].clear();
  }
  stats.clear();

  //initialize epsilon variable
  eps = params.initial_eps;
  eps_satisfied = INFINITECOST;

  //set MHA parameters
  //planner_type = PlannerType::SMHA;
  planner_type = PlannerType::IMHA;
  //meta_search_type = MetaSearchType::DTS;
  //meta_search_type = MetaSearchType::ROUND_ROBIN;
  meta_search_type = MetaSearchType::META_A_STAR;
  //DTS and Meta A*
  for(int i=0; i<num_heuristics; i++){
    queue_expands[i] = 0;
    queue_best_h[i] = INFINITECOST;
    meta_queue_best_h[i] = INFINITECOST;
    alpha[i] = 1.0;
    beta[i] = 1.0;
  }
  max_heur_dec = params.epsE * 1000; //TODO: total guess...

  //print the planner configuration
  if(meta_search_type == MetaSearchType::ROUND_ROBIN)
    printf("Round Robin ");
  else if(meta_search_type == MetaSearchType::META_A_STAR)
    printf("Meta-A* ");
  else if(meta_search_type == MetaSearchType::DTS)
    printf("DTS ");
  else
    ROS_ERROR("Meta Search approach is unknown!");
  if(planner_type == PlannerType::IMHA)
    printf("IMHA ");
  else if(planner_type == PlannerType::SMHA)
    printf("SMHA ");
  else
    ROS_ERROR("Planner type is unknown!");
  printf("with eps=%f and epsE=%f\n",eps,params.epsE);

  //initialize goal state
  goal_state.g = INFINITECOST;
  goal_state.v = INFINITECOST;
  goal_state.iteration_closed = -1;
  goal_state.replan_number = replan_number;
  goal_state.best_parent = NULL;
  goal_state.expanded_best_parent = NULL;
  goal_state.best_edge_type = EdgeType::NONE;
  goal_state.expanded_best_edge_type = EdgeType::NONE;
  goal_state.snap_midpoint = -1;
  goal_state.expanded_snap_midpoint = -1;
  goal_state.heapindex = 0;
  goal_state.in_incons = false;
  goal_state.isTrueCost = true;
  goal_state.h = 0; //h is always 0 at the goal

  for(int i=0; i<num_heuristics; i++)
    GetState(i, start_state_id);
  if(planner_type == PlannerType::SMHA)
    getAllHeuristics(start_state_id);

  //make a start state for each queue
  for(int i=0; i<num_heuristics; i++){
    start_state = GetState(i, start_state_id);
    start_state->g = 0;
    CKey key;
    key.key[0] = eps*start_state->h;
    heaps[i].insertheap(start_state, key);

    queue_best_h[i] = start_state->h;
    meta_queue_best_h[i] = start_state->h;
    assert(start_state->h >= 0);
  }

}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::initializeHeuristics(){
  exit(5);
  boost::unique_lock<boost::mutex> heuristic_lock(heuristic_mutex_);
  while(planner_ok_){
    //SLEEPING AND WAKING (enter under lock)
    sleeping_++;
    if(reported_for_duty_==num_threads_ && sleeping_==num_threads_){
      heuristics_initialized_ = true;
      work_id_ = 0;
      reported_for_duty_ = 0;
      main_thread_cond_.notify_one();
      //printf("last thread to sleep\n");
    }
    //printf("thread %d going to sleep\n",q_id);
    heuristic_cond_.wait(heuristic_lock);
    //printf("thread %d awake\n",q_id);
    sleeping_--;
    reported_for_duty_++;
    if(!planner_ok_)
      break;
    //SLEEPING AND WAKING (exit under lock)

    //DO WORK (enter under lock)
    while(work_id_ < num_heuristics){
      int q_id = work_id_++;
      heuristic_lock.unlock();
      int heur_val = egraph_mgr_->mha_egraph_heurs_[q_id]->getHeuristic(thread_heur_coord_);
      states[q_id][thread_heur_id_]->h = heur_val;
      heuristic_lock.lock();
    }
    //DO WORK (exit under lock)
  }
}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::getAllHeuristics(int state_id){
  double t0 = ros::Time::now().toSec();
  if (egraph_mgr_->egraph_env_->isGoal(state_id)){
    //loop over states and set to 0
    for(int i=0; i<num_heuristics; i++)
      states[i][state_id]->h = 0;
    double t1 = ros::Time::now().toSec();
    heuristicClock += t1-t0;
    return;
  }

  thread_heur_id_ = state_id;
  ContState cont_state;
  egraph_mgr_->egraph_env_->getCoord(state_id, cont_state);
  egraph_mgr_->egraph_env_->projectToHeuristicSpace(cont_state,thread_heur_coord_);
  /*
  //wake the threads!
  boost::unique_lock<boost::mutex> heuristic_lock(heuristic_mutex_);
  heuristics_initialized_ = false;
  heuristic_cond_.notify_all();
  //wait for them to finish
  while(!heuristics_initialized_)
    main_thread_cond_.wait(heuristic_lock);
    */
  
  for(int q_id=0; q_id<num_heuristics; q_id++){
    int heur_val = egraph_mgr_->mha_egraph_heurs_[q_id]->getHeuristic(thread_heur_coord_);
    states[q_id][thread_heur_id_]->h = heur_val;
  }

  double t1 = ros::Time::now().toSec();
  heuristicClock += t1-t0;
  //printf("heuristic for %d took %f\n",state_id,t1-t0);
  //std::cin.get();
}

template <typename HeuristicType>
bool MHEGraphPlanner<HeuristicType>::Search(vector<int>& pathIds, int& PathCost){
  CKey key;
  TimeStarted = ros::Time::now().toSec();

  initializeSearch();

  //the main loop of ARA*
  while(eps_satisfied > params.final_eps && !outOfTime()){

    //run weighted A*
    double before_time = ros::Time::now().toSec();
    int before_expands = search_expands;
    //ImprovePath returns:
    //1 if the solution is found
    //0 if the solution does not exist
    //2 if it ran out of time
    int ret = ImprovePath();
    if(ret == 1) //solution found for this iteration
      eps_satisfied = eps;
    int delta_expands = search_expands - before_expands;
    double delta_time = ros::Time::now().toSec()-before_time;

    //print the bound, expands, and time for that iteration
    printf("bound=%f expands=%d cost=%d time=%.2f\n", 
        eps_satisfied*params.epsE, delta_expands, goal_state.g, delta_time);

    //update stats
    totalExpands += delta_expands;
    PlannerStats tempStat;
    tempStat.eps = eps_satisfied;
    tempStat.expands = delta_expands;
    tempStat.time = delta_time;
    tempStat.cost = goal_state.g;
    stats.push_back(tempStat);

    //no solution exists
    if(ret == 0){
      printf("Solution does not exist\n");
      return false;
    }

    //if we're just supposed to find the first solution
    //or if we ran out of time, we're done
    if(params.return_first_solution || ret == 2)
      break;

    prepareNextSearchIteration();
  }

  if(goal_state.g == INFINITECOST){
    printf("could not find a solution (ran out of time)\n");
    return false;
  }
  if(eps_satisfied == INFINITECOST)
    printf("WARNING: a solution was found but we don't have quality bound for it!\n");

  printf("solution found\n");
  pathIds = GetSearchPath(PathCost);

  return true;
}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::prepareNextSearchIteration(){
  //decrease epsilon
  eps -= params.dec_eps;
  if(eps < params.final_eps)
    eps = params.final_eps;

  //dump the inconsistent states into the open list
  CKey key;
  for(int j=0; j<num_heuristics; j++){
    while(!incons[j].empty()){
      LazyAEGState* s = incons[j].back();
      incons[j].pop_back();
      s->in_incons = false;
      key.key[0] = s->g + int(eps * s->h);
      heaps[j].insertheap(s,key);
    }

    //recompute priorities for states in OPEN and reorder it
    for (int i=1; i<=heaps[j].currentsize; ++i){
      LazyAEGState* state = (LazyAEGState*)heaps[j].heap[i].heapstate;
      heaps[j].heap[i].key.key[0] = state->g + int(eps * state->h); 
    }
    heaps[j].makeheap();
  }

  search_iteration++;
}

template <typename HeuristicType>
int MHEGraphPlanner<HeuristicType>::GetBestHeuristicID(){
  if(meta_search_type == MetaSearchType::ROUND_ROBIN){
    // Round-robin
    static int heur_id = -1; //TODO: this must be reset on a new planning request
    heur_id = heur_id + 1;
    if (heur_id == num_heuristics)
    {
      heur_id = 0;
    }
    return heur_id;
  }
  else if(meta_search_type == MetaSearchType::META_A_STAR){
    // Meta-A*
    int best_id = -1;
    int best_priority = INFINITECOST;
    bool print = true;
    for (int ii = 0; ii < num_heuristics; ++ii)
    {
      const int priority = queue_expands[ii] + meta_queue_best_h[ii]/max_heur_dec;
      if (priority < best_priority)
      {
        best_priority = priority;
        best_id = ii;
      }
      if (print)
        printf("                      qid=%d g=%d h=%d f=%d (queue best h=%d)\n", ii,          queue_expands[ii], meta_queue_best_h[ii]/max_heur_dec, priority, meta_queue_best_h[ii]);
    }
    if(print)
      printf("%d\n", best_id);
    assert(best_id != -1);
    //std::cin.get();
    return best_id;
  }
  else if(meta_search_type == MetaSearchType::DTS){
    bool print = false;
    // Dynamic Thompson Sampling (DTS)
    //compute a random value from each beta distribution
    vector<double> rand_vals(num_heuristics,0);
    for(int i=0; i<num_heuristics; i++){
      /*
      if(queue_best_h[i]==0){
        printf("%d: done\n",i);
        rand_vals[i] = -1;
        continue;
      }
      */
      //beta_distribution<> dist(alpha[i], beta[i]);
      //double uniformRand = ((double)rand()/(double)RAND_MAX);
      //double betaRand = quantile(dist, uniformRand);
      double betaRand = gsl_ran_beta(gsl_rand, alpha[i], beta[i]);
      double betaMean = alpha[i]/(alpha[i]+beta[i]);
      if(print)
        printf("%d: alpha=%f beta=%f mean=%f betaRand=%f\n",i,alpha[i],beta[i],betaMean,         betaRand);
      rand_vals[i] = betaRand;
    }
    //find the best highest random value
    double best_rand = -1;
    for(int i=0; i<num_heuristics; i++)
      if(rand_vals[i] > best_rand)
        best_rand = rand_vals[i];
    //because of quantization we can get the exact same random value
    //for multiple queues more often than we'd like
    //especially when beta is very low (we get 1 very easily)
    //or when alpha is very low (we get 0 very easily)
    //in these cases, there is a bias toward the lower index queues
    //because they "improve" best_rand first
    //so when there are multiple queues near the best_rand value,
    //we will choose uniformly at random from them
    vector<int> near_best_rand;
    for(int i=0; i<num_heuristics; i++)
      if(fabs(best_rand-rand_vals[i]) < 0.0001)
        near_best_rand.push_back(i);
    //if(near_best_rand.size() > 1)
    //printf("choose uniformly among %lu queues\n",near_best_rand.size());
    int best_id = near_best_rand[rand()%near_best_rand.size()];

    if(print)
      printf("best=%d\n",best_id);
    //std::cin.get();
    assert(best_id != -1);
    return best_id;
  }
  else{
    ROS_ERROR("Unsupported meta search method!");
    assert(false);
    return -1;
  }
}


//-----------------------------Interface function-----------------------------------------------------

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::interrupt(){
  interruptFlag = true;
}

template <typename HeuristicType>
int MHEGraphPlanner<HeuristicType>::replan(vector<int>* solution_stateIDs_V, EGraphReplanParams p){
  int solcost = 0;
  return replan(solution_stateIDs_V, p, &solcost);
}

template <typename HeuristicType>
int MHEGraphPlanner<HeuristicType>::replan(int start, vector<int>* solution_stateIDs_V, EGraphReplanParams p, int* solcost){
  set_start(start);
  //set_goal(goal);
  return replan(solution_stateIDs_V, p, solcost);
}

template <typename HeuristicType>
int MHEGraphPlanner<HeuristicType>::replan(vector<int>* solution_stateIDs_V, EGraphReplanParams p, int* solcost){
  double replan_t0 = ros::Time::now().toSec();
  printf("planner: replan called\n");
  params = p;
  use_repair_time = params.repair_time >= 0;
  interruptFlag = false;

  egraph_mgr_->setEpsE(p.epsE);
  set_goal();

  if(start_state_id < 0){
    printf("ERROR searching: no start state set\n");
    return 0;
  }
  if (egraph_mgr_->egraph_env_->isGoal(start_state_id)){
    ROS_WARN("start is goal! nothing interesting returned");
    return true;
  }

  //plan
  vector<int> pathIds; 
  int PathCost;
  bool solnFound = Search(pathIds, PathCost);
  printf("max heuristic decrease found was %d\n",max_heur_dec);

  //copy the solution
  *solution_stateIDs_V = pathIds;
  *solcost = PathCost;

  start_state_id = -1;
  //goal_state_id = -1;

  double replan_t1 = ros::Time::now().toSec();
  totalPlanTime = replan_t1-replan_t0;

  printf("\n---------------------------------------------------------------\n");
  if(solnFound)
    printf("Solution found!\n");
  else
    printf("Solution not found...\n");
  printf("total time=%.2f total time without counting adding new path=%.2f\n", 
          totalPlanTime, totalPlanTime-feedbackPathTime);
  printf("total expands=%d solution cost=%d\n", 
          totalExpands, goal_state.g);
  printf("time breakdown: heuristic set goal  = %.2f\n", heuristicSetGoalTime);
  printf("                heuristic           = %.2f\n", heuristicClock);
  printf("                generate successors = %.2f\n", succsClock);
  printf("                shortcuts           = %.2f\n", shortcutClock);
  printf("                meta                = %.2f\n", metaTime);
  printf("                path reconstruction = %.2f\n", reconstructTime);
  printf("                feedback path       = %.2f\n", feedbackPathTime);
  printf("---------------------------------------------------------------\n\n");

  stat_map_["solution_found"] = solnFound;
  stat_map_["solution_bound"] = params.epsE*params.initial_eps;
  stat_map_["total_time"] = totalPlanTime;
  stat_map_["total_time_without_feedback"] = totalPlanTime-feedbackPathTime;
  stat_map_["expands"] = totalExpands;
  stat_map_["solution_cost"] = goal_state.g;
  stat_map_["heuristic_set_goal_time"] = heuristicSetGoalTime;
  stat_map_["heuristic_time"] = heuristicClock;
  stat_map_["generate_successors_time"] = succsClock;
  stat_map_["shortcuts_time"] = shortcutClock;
  stat_map_["path_reconstruction_time"] = reconstructTime;
  stat_map_["feedback_path_time"] = feedbackPathTime;
         
  return (int)solnFound;
}

template <typename HeuristicType>
int MHEGraphPlanner<HeuristicType>::set_goal(){
  /*
  printf("planner: setting goal to %d\n", id);
  if(bforwardsearch)
    goal_state_id = id;
  else
    start_state_id = id;
  */
  if (!params.use_lazy_validation){
      ROS_INFO("fully validating egraph");
      egraph_mgr_->validateEGraph();
  }
  double t0 = ros::Time::now().toSec();
  egraph_mgr_->setGoal();
  double t1 = ros::Time::now().toSec();
  heuristicSetGoalTime = t1-t0;

  if (!params.use_lazy_validation){
      egraph_mgr_->initEGraph();
  }

  return 1;
}

template <typename HeuristicType>
int MHEGraphPlanner<HeuristicType>::set_start(int id){
  //printf("planner: setting start to %d\n", id);
  //if(bforwardsearch)
    start_state_id = id;
  //else
    //goal_state_id = id;
  return 1;
}

template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::feedback_last_path(){
    egraph_mgr_->feedbackLastPath();
    printf("validitycheck time=%.3f feedbacktime %.3f errorcheck time=%.3f\n",
            egraph_mgr_->getStats().egraph_validity_check_time,
            egraph_mgr_->getStats().feedback_time,
            egraph_mgr_->getStats().error_check_time);
}


//---------------------------------------------------------------------------------------------------------


template <typename HeuristicType>
void MHEGraphPlanner<HeuristicType>::get_search_stats(vector<PlannerStats>* s){
  s->clear();
  s->reserve(stats.size());
  for(unsigned int i=0; i<stats.size(); i++){
    s->push_back(stats[i]);
  }
}

