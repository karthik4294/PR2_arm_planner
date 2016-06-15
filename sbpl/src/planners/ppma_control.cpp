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
#include <sbpl/planners/ppma_control.h>

#include "/usr/local/include/ompl/base/ProblemDefinition.h"

#include <utility>

using namespace std;
using namespace ompl;

// PPMAControlPlanner::PPMAControlPlanner(const ompl::base::SpaceInformationPtr &si,
//                          EnvironmentPPMA *environment,
//                          bool bSearchForward) : ompl::base::Planner(si, "ppma_planner"),
// params(0.0) {
//   bforwardsearch = bSearchForward;
//   env_ = environment;
//   replan_number = 0;

//   goal_state_id = -1;
//   start_state_id = -1;

//   // OMPL Stuff
//   InitializeOMPL();
// }

PPMAControlPlanner::PPMAControlPlanner(const ompl::control::SpaceInformationPtr &si,
                                       EnvironmentPPMA *environment,
                                       bool bSearchForward, double alloc_time, ReplanParams* get_params) : ompl::base::Planner(si, "ppma_control_planner"),
    params(0.0), replan_params(alloc_time) {
    bforwardsearch = bSearchForward;
    env_ = environment;
    replan_number = 0;

    goal_state_id = -1;
    start_state_id = -1;

    // OMPL Stuff
    InitializeOMPL();

    replan_params.initial_eps = get_params->initial_eps;
    replan_params.final_eps = get_params->final_eps;
    replan_params.return_first_solution = get_params->return_first_solution;

    si_c_ = si.get();

}

void PPMAControlPlanner::InitializeOMPL() {
    specs_.approximateSolutions = true;
    // specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    addIntermediateStates_ = false;
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("goal_bias", this, &PPMAControlPlanner::setGoalBias,
                                  &PPMAControlPlanner::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &PPMAControlPlanner::setIntermediateStates, &PPMAControlPlanner::getIntermediateStates);
    setup();
    // for (int i = 0; i < 10; ++i) {
    //  printf("Random num: %f\n", rng_.uniform01());
    // }
}

PPMAControlPlanner::~PPMAControlPlanner() {
    freeMemory();
}

PPMAControlState *PPMAControlPlanner::GetState(int id) {
    //if this stateID is out of bounds of our state vector then grow the list
    if (id >= int(states.size())) {
        for (int i = states.size(); i <= id; i++) {
            states.push_back(NULL);
        }
    }

    //if we have never seen this state then create one
    if (states[id] == NULL) {
        states[id] = new PPMAControlState();
        states[id]->id = id;
        states[id]->replan_number = -1;
    }

    //initialize the state if it hasn't been for this call to replan
    PPMAControlState *s = states[id];

    if (s->replan_number != replan_number) {
        s->g = INFINITECOST;
        s->v = INFINITECOST;
        s->iteration_closed = -1;
        s->replan_number = replan_number;
        s->best_parent = NULL;
        s->expanded_best_parent = NULL;
        s->heapindex = 0;
        s->in_incons = false;
        s->isTrueCost = true;

        //clear the lazy list
        while (!s->lazyList.empty()) {
            s->lazyList.pop();
        }

        //compute heuristics
        if (bforwardsearch) {
            s->h = env_->GetGoalHeuristic(s->id);
        } else {
            s->h = env_->GetStartHeuristic(s->id);
        }
    }

    return s;
}

void PPMAControlPlanner::ExpandState(PPMAControlState *parent) {
    bool print = false; //parent->id == 285566;

    if (print) {
        ///printf("expand %d\n", parent->id);
    }

    vector<int> children;
    vector<int> costs;
    vector<bool> isTrueCost;

    if (bforwardsearch) {
        env_->GetLazySuccs(parent->id, &children, &costs, &isTrueCost);
    } else {
        env_->GetLazyPreds(parent->id, &children, &costs, &isTrueCost);
    }

    //printf("expand %d\n",parent->id);
    //iterate through children of the parent
    for (int i = 0; i < (int)children.size(); i++) {
        //printf("  succ %d\n",children[i]);
        PPMAControlState *child = GetState(children[i]);
        insertLazyList(child, parent, costs[i], isTrueCost[i]);
    }
}

//assumptions:
//state is at the front of the open list
//it's minimum f-value is an underestimate (the edge cost from the parent is a guess and needs to be evaluated properly)
//it hasn't been expanded yet this iteration
void PPMAControlPlanner::EvaluateState(PPMAControlState *state) {
    bool print = false; //state->id == 285566;

    if (print) {
        ///printf("evaluate %d (from %d)\n", state->id, state->best_parent->id);
    }

    PPMAControlState *parent = state->best_parent;

    getNextLazyElement(state);
    //printf("state_ptr=%p\n",state);
    //printf("state_id=%d\n",state->id);
    //printf("parent_ptr=%p\n",parent);
    //printf("parent_id=%d\n",parent->id);
    int trueCost = env_->GetTrueCost(parent->id, state->id);

    //printf("has a true cost of %d\n",trueCost);
    if (trueCost >
            0) { //if the evaluated true cost is valid (positive), insert it into the lazy list
        insertLazyList(state, parent, trueCost, true);
    }
}

//this should only be used with EvaluateState since it is assuming state hasn't been expanded yet (only evaluated)
void PPMAControlPlanner::getNextLazyElement(PPMAControlState *state) {
    if (state->lazyList.empty()) {
        state->g = INFINITECOST;
        state->best_parent = NULL;
        state->isTrueCost = true;
        return;
    }

    PPMAControlLazyListElement elem = state->lazyList.top();
    state->lazyList.pop();
    state->g = elem.parent->v + elem.edgeCost;
    state->best_parent = elem.parent;
    state->isTrueCost = elem.isTrueCost;

    //the new value is cheapest and if the value is also true then we want to throw out all the other options
    if (state->isTrueCost) {
        while (!state->lazyList.empty()) {
            state->lazyList.pop();
        }
    }

    putStateInHeap(state);
}

void PPMAControlPlanner::insertLazyList(PPMAControlState *state, PPMAControlState *parent,
                                        int edgeCost, bool isTrueCost) {
    bool print = true; //state->id == 285566 || parent->id == 285566;

    if (print) {
        printf("state->id=%d state->g=%d parent->v=%d edgeCost=%d isTrueCost=%d\n",
        state->id, state->g, parent->v, edgeCost, isTrueCost);
    }

    if (state->v <= parent->v + edgeCost) {
        return;
    } else if (state->g <= parent->v + edgeCost) {
        //if the best g-value we have is true and better, then the value we had dominates this one and we don't need it
        if (state->isTrueCost) {
            return;
        }

        //insert this guy into the lazy list
        PPMAControlLazyListElement elem(parent, edgeCost, isTrueCost);
        state->lazyList.push(elem);
    } else { //the new guy is the cheapest so far
        //should we save what was the previous best?
        if (!isTrueCost && //the better guy's cost is not for sure
                //state->g < INFINITECOST && //we actually have a previous best (we actually don't need this line because of the next one)
                state->g <
                state->v) { //we're not saving something we already expanded (and is stored in v and expanded_best_parent)
            //we save it by putting it in the lazy list
            PPMAControlLazyListElement elem(state->best_parent, state->g - state->best_parent->v,
                                            state->isTrueCost);
            state->lazyList.push(elem);
            //printf("save the previous best\n");
        }

        //the new guy is the cheapest
        state->g = parent->v + edgeCost;
        state->best_parent = parent;
        state->isTrueCost = isTrueCost;

        //the new value is cheapest and if the value is also true then we want to throw out all the other options
        if (isTrueCost) {
            //printf("clear the lazy list\n");
            while (!state->lazyList.empty()) {
                state->lazyList.pop();
            }
        }

        //this function puts the state into the heap (or updates the position) if we haven't expanded
        //if we have expanded, it will put the state in the incons list (if we haven't already)
        putStateInHeap(state);
    }
}

void PPMAControlPlanner::putStateInHeap(PPMAControlState *state) {
    bool print = false; //state->id == 285566;
    if (state->id == goal_state_id) {
        //printf("putting goal in heap\n");
    }

    //we only allow one expansion per search iteration
    //so insert into heap if not closed yet
    if (state->iteration_closed != search_iteration) {
        if (print) {
            ///printf("put state in open\n");
        }

        CKey key;
        key.key[0] = state->g + int(eps * state->h);

        //if the state is already in the heap, just update its priority
        if (state->heapindex != 0) {
            heap.updateheap(state, key);
        } else { //otherwise add it to the heap
            heap.insertheap(state, key);
        }

        // Add to the monolithic tree. TODO(venkat): since we add to the tree even
        // for updateheap, we should update the tree as well by deleting the old
        // motion.
        AddToMonolithicTree(state);
    }
    //if the state has already been expanded once for this iteration
    //then add it to the incons list so we can keep track of states
    //that we know we have better costs for
    else if (!state->in_incons) {
        if (print) {
            ///printf("put state in incons\n");
        }

        incons.push_back(state);
        state->in_incons = true;
    }
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int PPMAControlPlanner::ImprovePath() {

    //expand states until done
    int expands = 0;
    CKey min_key = heap.getminkeyheap();

    // OMPL Stuff
    ControlMotion *solution  = NULL;
    ControlMotion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    ControlMotion *rmotion   = new ControlMotion(si_c_);
    base::State *rstate = rmotion->state;
    ompl::control::Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();
    base::State *nearest_lattice_state = si_->allocState();

    // while (!heap.emptyheap() &&
    //        min_key.key[0] < INFINITECOST &&
    //        (goal_state->g > min_key.key[0] || !goal_state->isTrueCost) &&
    //        !outOfTime()) {
    while (!outOfTime()) {
        if (goal_state->g <= min_key.key[0] && goal_state->isTrueCost) {
            break;
        }

        //get the state
        PPMAControlState *state = (PPMAControlState *)heap.deleteminheap();

        if (state->v == state->g) {
            ////printf("ERROR: consistent state is being expanded\n");
            ///printf("id=%d v=%d g=%d isTrueCost=%d lazyListSize=%lu\n",
            ///state->id, state->v, state->g, state->isTrueCost, state->lazyList.size());
            throw new SBPL_Exception();
        }

        if (state->isTrueCost) {
            //mark the state as expanded
            state->v = state->g;
            state->expanded_best_parent = state->best_parent;
            state->iteration_closed = search_iteration;
            //expand the state
            expands++;
            ExpandState(state);
            //printf("Expanded %d with heuristic %d\n", state->id, state->h);

            if (state->best_parent) {
                base::State *continuous_state = si_->allocState();
                env_->GetContState(state->id, continuous_state);
                base::State *continuous_parent_state = si_->allocState();
                env_->GetContState(state->best_parent->id, continuous_parent_state);
                env_->VisualizeContState(continuous_state, continuous_parent_state, true, false);
            }

            if (expands % 100000 == 0) {
                ///printf("expands so far=%u\n", expands);
            }
        } else { //otherwise the state needs to be evaluated for its true cost
            EvaluateState(state);
        }


        bool explore_rrt = true;
        // Explore with RRT (other planners go here)

        if (explore_rrt) {
            /* sample random state (with goal biasing) */
            if (goal_s_ && rng_.uniform01() < goalBias_ && goal_s_->canSample()) {
                goal_s_->sampleGoal(rstate);
            } else {
                sampler_->sampleUniform(rstate);
            }

            /* find closest state in the tree */
            ControlMotion *nmotion = monolithic_tree_->nearest(rmotion);

            /* sample a random control that attempts to go towards the random state, and also sample a control duration */
            unsigned int cd = control_sampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

            std::vector<base::State *> pstates;
            cd = si_c_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= si_c_->getMinControlDuration())
            {
                ControlMotion *lastmotion = nmotion;
                size_t p = 0;

                for( ; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    ControlMotion *motion = new ControlMotion(si_c_);
                    si_->copyState(motion->state, pstates[p]);//rmotion->state
                    si_c_->copyControl(motion->control, rctrl);
                    motion->steps = 1;//cd
                    motion->parent = lastmotion;//nmotion

                    // TODO: We should probably pass in the intermediate states as well
                    // to compute the edge cost under the kinodynamic constraints.
                    int tree_edge_cost = env_->GetContEdgeCost(lastmotion->state, motion->state);//nmotion->state
                    motion->g = tree_edge_cost + lastmotion->g;//nmotion->g
                    monolithic_tree_->add(motion);

                    // Visualize latest state added to tree
                    // TODO: interm states here as well
                    env_->VisualizeContState(motion->state,lastmotion->state, false, false);//nmotion->state

                    lastmotion = motion;

                    double dist = 0.0;
                    bool solv = goal_s_->isSatisfied(motion->state, &dist);
                    //printf("Dist to goal: %f\n", dist);
                    if (solv)
                    {
                        approxdif = dist;
                        solution = motion;
                        goal_state->g = std::min(goal_state->g, static_cast<unsigned int>(motion->g));
                        goal_state->isTrueCost = true;
                        ROS_INFO("RRT found goal!");
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }

                while (++p < pstates.size())
                    si_->freeState(pstates[p]);

                // Now find the closest lattice state in the monolithic tree and
                // 'extend' it using a motion primitive.
                ControlMotion *nearest_lattice_motion = lattice_tree_->nearest(rmotion);

                int nearest_lattice_state_id = -1;
                env_->GetNearestLatticeState(nearest_lattice_motion->state, nearest_lattice_state, &nearest_lattice_state_id);
                printf("Found nearest lattice state %d!\n", nearest_lattice_state_id);
                PPMAControlState *lattice_state = GetState(nearest_lattice_state_id);
                if (lattice_state->id != start_state_id && !lattice_state->expanded_best_parent) {
                    printf("The nearest lattice state %d in the tree does not have a valid parent!\n", lattice_state->id);
                }

                // We should not mark this state as 'expanded'.
                vector<int> children;
                vector<int> costs;
                vector<bool> isTrueCost;
                if (bforwardsearch) {
                    env_->GetLazySuccs(lattice_state->id, &children, &costs, &isTrueCost);
                } else {
                    env_->GetLazyPreds(lattice_state->id, &children, &costs, &isTrueCost);
                }



                //printf("expand %d\n",parent->id);
                //iterate through children of the parent
                // TODO(venkat): we should really add just the closest successor to
                // the open list, rather than all successors.
                for (int i = 0; i < (int)children.size(); i++) {
                    //printf("  succ %d\n",children[i]);
                    PPMAControlState *child = GetState(children[i]);
                    insertLazyList(child, lattice_state, costs[i], isTrueCost[i]);

                    base::State *continuous_state = si_->allocState();
                    env_->GetContState(child->id, continuous_state);
                    base::State *continuous_parent_state = si_->allocState();
                    env_->GetContState(lattice_state->id, continuous_parent_state);
                    env_->VisualizeContState(continuous_state, continuous_parent_state, true, false);
                    si_->freeState(continuous_parent_state);
                    si_->freeState(continuous_state);

                }
            }
            else {
                for (size_t p = 0 ; p < pstates.size(); ++p)
                    si_->freeState(pstates[p]);
            }
        }

        //get the min key for the next iteration
        min_key = heap.getminkeyheap();
        //printf("min_key =%d\n",min_key.key[0]);
    }

    search_expands += expands;

    if (goal_state->g == INFINITECOST && (heap.emptyheap() ||
                                          min_key.key[0] >= INFINITECOST)) {
        return 0;  //solution does not exists
    }

    if (!heap.emptyheap() && goal_state->g > min_key.key[0]) {
        return 2;  //search exited because it ran out of time
    }

    ///printf("search exited with a solution for eps=%.3f\n", eps);

    if (goal_state->g < goal_state->v) {
        goal_state->expanded_best_parent = goal_state->best_parent;
        goal_state->v = goal_state->g;
    }

    bool solved = false;
    bool approximate = false;
    if (lastGoalMotion_ == NULL) {
        lastGoalMotion_ = approxsol;
        approximate = true;
    }

    if (lastGoalMotion_ != NULL) {
        // lastGoalMotion_ = solution;
        solution = lastGoalMotion_;

        /* construct the solution path */
        std::vector<ControlMotion *> mpath;

        while (solution != NULL) {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        ompl::control::PathControl *path = new ompl::control::PathControl(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i) {
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * si_c_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
            if(i != 0)
                env_->VisualizeContState(mpath[i]->state,mpath[i-1]->state, false, true);
            else
                env_->VisualizeContState(mpath[i]->state,mpath[i]->state, false, true);
        }
        solved = true;
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());

        ///printf("SOLVED!!! Path has %zu waypoints\n", mpath.size());
    }


    si_->freeState(xstate);
    si_->freeState(nearest_lattice_state);

    if (rmotion->state) {
        si_->freeState(rmotion->state);
    }

    delete rmotion;
    OMPL_INFORM("%s: Created %u states", getName().c_str(),
                monolithic_tree_->size());
    // return base::PlannerStatus(solved, approximate);

    return 1;
}

vector<int> PPMAControlPlanner::GetSearchPath(int &solcost) {
    vector<int> SuccIDV;
    vector<int> CostV;
    vector<bool> isTrueCost;
    vector<int> wholePathIds;

    PPMAControlState *state;
    PPMAControlState *final_state;

    if (bforwardsearch) {
        state = goal_state;
        final_state = start_state;
    } else {
        state = start_state;
        final_state = goal_state;
    }

    wholePathIds.push_back(state->id);
    solcost = 0;

    while (state->id != final_state->id) {
        if (state->expanded_best_parent == NULL) {
            ///printf("a state along the path has no parent!\n");
            break;
        }

        if (state->v == INFINITECOST) {
            ///printf("a state along the path has an infinite g-value!\n");
            ///printf("inf state = %d\n", state->id);
            break;
        }

        if (bforwardsearch) {
            env_->GetLazySuccs(state->expanded_best_parent->id, &SuccIDV, &CostV,
                               &isTrueCost);
        } else {
            env_->GetLazyPreds(state->expanded_best_parent->id, &SuccIDV, &CostV,
                               &isTrueCost);
        }

        int actioncost = INFINITECOST;

        //printf("reconstruct expand %d\n",state->expanded_best_parent->id);
        for (unsigned int i = 0; i < SuccIDV.size(); i++) {
            //printf("  succ %d\n",SuccIDV[i]);
            if (SuccIDV[i] == state->id && CostV[i] < actioncost) {
                actioncost = CostV[i];
            }
        }

        if (actioncost == INFINITECOST) {
            ///printf("WARNING: actioncost = %d\n", actioncost);
        }

        solcost += actioncost;

        state = state->expanded_best_parent;
        wholePathIds.push_back(state->id);
    }

    //if we searched forward then the path reconstruction
    //worked backward from the goal, so we have to reverse the path
    if (bforwardsearch) {
        //in place reverse
        for (unsigned int i = 0; i < wholePathIds.size() / 2; i++) {
            int other_idx = wholePathIds.size() - i - 1;
            int temp = wholePathIds[i];
            wholePathIds[i] = wholePathIds[other_idx];
            wholePathIds[other_idx] = temp;
        }
    }

    return wholePathIds;
}

bool PPMAControlPlanner::outOfTime() {
    //if the user has sent an interrupt signal we stop
    if (interruptFlag) {
        return true;
    }

    //if we are supposed to run until the first solution, then we are never out of time
    if (params.return_first_solution) {
        return false;
    }

    double time_used = double(clock() - TimeStarted) / CLOCKS_PER_SEC;

    if (time_used >= params.max_time) {
        ///printf("out of max time\n");
    }

    if (use_repair_time && eps_satisfied != INFINITECOST &&
            time_used >= params.repair_time) {
        ///printf("used all repair time...\n");
    }

    //we are out of time if:
    //we used up the max time limit OR
    //we found some solution and used up the minimum time limit
    return time_used >= params.max_time ||
           (use_repair_time && eps_satisfied != INFINITECOST &&
            time_used >= params.repair_time);
}

void PPMAControlPlanner::initializeSearch() {
    //it's a new search, so increment replan_number and reset the search_iteration
    replan_number++;
    search_iteration = 0;
    search_expands = 0;
    totalPlanTime = 0;
    totalExpands = 0;

    //clear open list, incons list, and stats list
    heap.makeemptyheap();
    incons.clear();
    stats.clear();

    //initialize epsilon variable
    eps = params.initial_eps;
    eps_satisfied = INFINITECOST;

    //call get state to initialize the start and goal states
    goal_state = GetState(goal_state_id);
    start_state = GetState(start_state_id);

    //put start state in the heap
    start_state->g = 0;
    CKey key;
    key.key[0] = eps * start_state->h;
    heap.insertheap(start_state, key);

    //ensure heuristics are up-to-date
    env_->EnsureHeuristicsUpdated((bforwardsearch == true));

    // OMPL stuff
    checkValidity();
    goal_ = pdef_->getGoal().get();
    goal_s_ = dynamic_cast<base::GoalSampleableRegion *>(goal_);

    while (const base::State *st = pis_.nextStart()) {
        ControlMotion *motion = new ControlMotion(si_c_);
        si_->copyState(motion->state, st);
        si_c_->nullControl(motion->control);
        motion->g = 0;
        monolithic_tree_->add(motion);
        lattice_tree_->add(motion);
        env_->VisualizeContState(motion->state,motion->state,false, false);
    }

    if (monolithic_tree_->size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return;
        // return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_) {
        sampler_ = si_->allocStateSampler();
    }
    if (!control_sampler_) {
        control_sampler_ = si_c_->allocDirectedControlSampler();
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
                getName().c_str(), monolithic_tree_->size());

}

bool PPMAControlPlanner::Search(vector<int> &pathIds, int &PathCost) {
    CKey key;
    TimeStarted = clock();

    initializeSearch();

    //the main loop of PPMA
    while (eps_satisfied > params.final_eps && !outOfTime()) {

        //run weighted A*
        clock_t before_time = clock();
        int before_expands = search_expands;
        //ImprovePath returns:
        //1 if the solution is found
        //0 if the solution does not exist
        //2 if it ran out of time
        int ret = ImprovePath();

        if (ret == 1) { //solution found for this iteration
            eps_satisfied = eps;
        }

        int delta_expands = search_expands - before_expands;
        double delta_time = double(clock() - before_time) / CLOCKS_PER_SEC;

        //print the bound, expands, and time for that iteration
        ///printf("bound=%f expands=%d cost=%d time=%.3f\n",
        ///eps_satisfied, delta_expands, goal_state->g, delta_time);

        //update stats
        totalPlanTime += delta_time;
        totalExpands += delta_expands;
        PlannerStats tempStat;
        tempStat.eps = eps_satisfied;
        tempStat.expands = delta_expands;
        tempStat.time = delta_time;
        tempStat.cost = goal_state->g;
        stats.push_back(tempStat);

        //no solution exists
        if (ret == 0) {
            ///printf("Solution does not exist\n");
            return false;
        }

        //if we're just supposed to find the first solution
        //or if we ran out of time, we're done
        if (params.return_first_solution || ret == 2) {
            break;
        }

        prepareNextSearchIteration();
    }

    if (goal_state->g == INFINITECOST) {
        ///printf("could not find a solution (ran out of time)\n");
        return false;
    }

    if (eps_satisfied == INFINITECOST) {
        ///printf("WARNING: a solution was found but we don't have quality bound for it!\n");
    }

    ///printf("solution found\n");
    clock_t before_reconstruct = clock();
    pathIds = GetSearchPath(PathCost);
    reconstructTime = double(clock() - before_reconstruct) / CLOCKS_PER_SEC;
    totalTime = totalPlanTime + reconstructTime;

    return true;
}

void PPMAControlPlanner::prepareNextSearchIteration() {
    //decrease epsilon
    eps -= params.dec_eps;

    if (eps < params.final_eps) {
        eps = params.final_eps;
    }

    //dump the inconsistent states into the open list
    CKey key;

    while (!incons.empty()) {
        PPMAControlState *s = incons.back();
        incons.pop_back();
        s->in_incons = false;
        key.key[0] = s->g + int(eps * s->h);
        heap.insertheap(s, key);
    }

    //recompute priorities for states in OPEN and reorder it
    for (int i = 1; i <= heap.currentsize; ++i) {
        PPMAControlState *state = (PPMAControlState *)heap.heap[i].heapstate;
        heap.heap[i].key.key[0] = state->g + int(eps * state->h);
    }

    heap.makeheap();

    search_iteration++;
}


//-----------------------------Interface function-----------------------------------------------------

void PPMAControlPlanner::interrupt() {
    interruptFlag = true;
}

int PPMAControlPlanner::replan(vector<int> *solution_stateIDs_V, ReplanParams p) {
    int solcost;
    return replan(solution_stateIDs_V, p, &solcost);
}

int PPMAControlPlanner::replan(int start, int goal, vector<int> *solution_stateIDs_V,
                               ReplanParams p, int *solcost) {
    set_start(start);
    set_goal(goal);
    return replan(solution_stateIDs_V, p, solcost);
}

int PPMAControlPlanner::replan(vector<int> *solution_stateIDs_V, ReplanParams p,
                               int *solcost) {
    ///printf("planner: replan called\n");
    params = p;
    use_repair_time = params.repair_time >= 0;
    interruptFlag = false;

    if (goal_state_id < 0) {
        ///printf("ERROR searching: no goal state set\n");
        return 0;
    }

    if (start_state_id < 0) {
        ///printf("ERROR searching: no start state set\n");
        return 0;
    }

    //plan
    vector<int> pathIds;
    int PathCost;
    bool solnFound = Search(pathIds, PathCost);
    ///printf("total expands=%d planning time=%.3f reconstruct path time=%.3f total time=%.3f solution cost=%d\n",
    ///totalExpands, totalPlanTime, reconstructTime, totalTime, goal_state->g);

    //copy the solution
    *solution_stateIDs_V = pathIds;
    *solcost = PathCost;

    start_state_id = -1;
    goal_state_id = -1;

    return (int)solnFound;
}

int PPMAControlPlanner::set_goal(int id) {
    ///printf("planner: setting goal to %d\n", id);

    if (bforwardsearch) {
        goal_state_id = id;
    } else {
        start_state_id = id;
    }

    return 1;
}

int PPMAControlPlanner::set_start(int id) {
    ///printf("planner: setting start to %d\n", id);

    if (bforwardsearch) {
        start_state_id = id;
    } else {
        goal_state_id = id;
    }

    return 1;
}

//--------------------------------------------------------------------------------------------------------
// OMPL stuff
void PPMAControlPlanner::clear() {
    Planner::clear();
    sampler_.reset();
    control_sampler_.reset();
    freeMemory();

    if (monolithic_tree_) {
        monolithic_tree_->clear();
    }

    if (lattice_tree_) {
        lattice_tree_->clear();
    }

    lastGoalMotion_ = NULL;
}

void PPMAControlPlanner::setup() {
    base::Planner::setup();

    if (!monolithic_tree_) {
        monolithic_tree_.reset(tools::SelfConfig::getDefaultNearestNeighbors<ControlMotion *>(this));
    }

    monolithic_tree_->setDistanceFunction(boost::bind(
            &PPMAControlPlanner::distanceFunction, this, _1,
            _2));

    if (!lattice_tree_) {
        lattice_tree_.reset(tools::SelfConfig::getDefaultNearestNeighbors<ControlMotion *>(this));
    }

    lattice_tree_->setDistanceFunction(boost::bind(
                                           &PPMAControlPlanner::distanceFunction, this, _1,
                                           _2));
}

void PPMAControlPlanner::freeMemory() {
    if (monolithic_tree_) {
        std::vector<ControlMotion *> motions;
        monolithic_tree_->list(motions);

        for (unsigned int i = 0 ; i < motions.size() ; ++i) {
            if (motions[i]->state) {
                si_->freeState(motions[i]->state);
            }
            if (motions[i]->control) {
                si_c_->freeControl(motions[i]->control);
            }

            delete motions[i];
        }
    }

    if (lattice_tree_) {
        std::vector<ControlMotion *> motions;
        lattice_tree_->list(motions);

        for (unsigned int i = 0 ; i < motions.size() ; ++i) {
            if (motions[i]->state) {
                si_->freeState(motions[i]->state);
            }
            if (motions[i]->control) {
                si_c_->freeControl(motions[i]->control);
            }

            delete motions[i];
        }
    }
}

base::PlannerStatus PPMAControlPlanner::solve(const
        base::PlannerTerminationCondition &ptc) {

    bool solved = false;
    bool approximate = false;

    int bRet = replan(&solution_stateIDs_V,replan_params);

    ///printf("done planning\n");
    ///printf("size of solution=%d\n", (unsigned int)solution_stateIDs_V.size());

    if(bRet) solved = true;

    // checkValidity();
    // base::Goal                 *goal   = pdef_->getGoal().get();
    // base::GoalSampleableRegion *goal_s =
    //   dynamic_cast<base::GoalSampleableRegion *>(goal);

    // while (const base::State *st = pis_.nextStart()) {
    //   ControlMotion *motion = new ControlMotion(si_);
    //   si_->copyState(motion->state, st);
    //   monolithic_tree_->add(motion);
    // }

    // if (monolithic_tree_->size() == 0) {
    //   OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    //   return base::PlannerStatus::INVALID_START;
    // }

    // if (!sampler_) {
    //   sampler_ = si_->allocStateSampler();
    // }

    // OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
    //             getName().c_str(), monolithic_tree_->size());

    // ControlMotion *solution  = NULL;
    // ControlMotion *approxsol = NULL;
    // double  approxdif = std::numeric_limits<double>::infinity();
    // ControlMotion *rmotion   = new ControlMotion(si_);
    // base::State *rstate = rmotion->state;
    // base::State *xstate = si_->allocState();

    // while (ptc == false) {

    //   /* sample random state (with goal biasing) */
    //  if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
    //     goal_s->sampleGoal(rstate);
    //   } else {
    //     sampler_->sampleUniform(rstate);
    //   }

    //   /* find closest state in the tree */
    //   ControlMotion *nmotion = monolithic_tree_->nearest(rmotion);
    //   base::State *dstate = rstate;

    //   /* find state to add */
    //   double d = si_->distance(nmotion->state, rstate);

    //   if (d > maxDistance_) {
    //     si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d,
    //                                       xstate);
    //     dstate = xstate;
    //   }

    //   if (si_->checkControlMotion(nmotion->state, dstate)) {
    //     /* create a motion */
    //     ControlMotion *motion = new ControlMotion(si_);
    //     si_->copyState(motion->state, dstate);
    //     motion->parent = nmotion;

    //     monolithic_tree_->add(motion);
    //     double dist = 0.0;
    //     bool sat = goal->isSatisfied(motion->state, &dist);

    //     if (sat) {
    //       approxdif = dist;
    //       solution = motion;
    //       break;
    //     }

    //     if (dist < approxdif) {
    //       approxdif = dist;
    //       approxsol = motion;
    //     }
    //   }
    // }

    // bool solved = false;
    // bool approximate = false;

    // if (solution == NULL) {
    //   solution = approxsol;
    //   approximate = true;
    // }

    // if (solution != NULL) {
    //   lastGoalMotion_ = solution;

    //   /* construct the solution path */
    //   std::vector<ControlMotion *> mpath;

    //   while (solution != NULL) {
    //     mpath.push_back(solution);
    //     solution = solution->parent;
    //   }

    //   /* set the solution path */
    //   ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(si_);

    //   for (int i = mpath.size() - 1 ; i >= 0 ; --i) {
    //     path->append(mpath[i]->state);
    //   }

    //   pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
    //   solved = true;
    // }

    // si_->freeState(xstate);

    // if (rmotion->state) {
    //   si_->freeState(rmotion->state);
    // }

    // delete rmotion;

    // OMPL_INFORM("%s: Created %u states", getName().c_str(),
    //             monolithic_tree_->size());

    return base::PlannerStatus(solved, false);
}

void PPMAControlPlanner::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);

    std::vector<ControlMotion *> motions;

    if (monolithic_tree_) {
        monolithic_tree_->list(motions);
    }

    double delta = si_c_->getPropagationStepSize();

    if (lastGoalMotion_) {
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
    }

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        const ControlMotion *m = motions[i];
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                             base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                             base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}

void PPMAControlPlanner::AddToMonolithicTree(PPMAControlState *state) {

    // NOTE: Assuming that a state without a backpointer has a continuous parent,
    // and has already been added to the tree.
    // Hence we will omit adding this to the tree at this point.

    if (!state->best_parent) {
        return;
    }

    ControlMotion *parent_motion   = new ControlMotion(si_c_);

    base::State *continuous_parent_state = si_->allocState();
    env_->GetContState(state->best_parent->id, continuous_parent_state);

    // args are ordered by (destination, source), ugh.
    si_->copyState(parent_motion->state, continuous_parent_state);

    /* find closest state in the tree */
    ControlMotion *nearest_motion = monolithic_tree_->nearest(parent_motion);

    /* create the new motion */
    ControlMotion *motion = new ControlMotion(si_c_);
    base::State *continuous_child_state = si_->allocState();
    env_->GetContState(state->best_parent->id, continuous_child_state);
    si_->copyState(motion->state, continuous_child_state);
    motion->parent = nearest_motion;

    int tree_edge_cost = env_->GetContEdgeCost(nearest_motion->state, motion->state);
    motion->g = tree_edge_cost + nearest_motion->g;
    motion->lattice_state = true;
    lattice_tree_->add(motion);

    monolithic_tree_->add(motion);
    //env_->VisualizeContState(motion->state,nearest_motion->state);

    // TODO(venkat): this will break if we have an underspecified goal.
    if (state->id == goal_state_id) {
        //printf("Goal state g-val: %d, has parent: %d with g-val %d\n", state->g, state->best_parent->id, motion->parent->g);
        lastGoalMotion_ = motion;
    }
}

//---------------------------------------------------------------------------------------------------------

void PPMAControlPlanner::get_search_stats(vector<PlannerStats> *s) {
    s->clear();
    s->reserve(stats.size());

    for (unsigned int i = 0; i < stats.size(); i++) {
        s->push_back(stats[i]);
    }
}

