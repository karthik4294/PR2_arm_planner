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
#include <sbpl/planners/ppma.h>

#include "/usr/local/include/ompl/base/ProblemDefinition.h"
#include "/usr/local/include/ompl/tools/config/SelfConfig.h"
#include "/usr/local/include/ompl/geometric/PathGeometric.h"

#include <utility>

#include <chrono>

using namespace std;
using namespace ompl;

int soln_cost;

// PPMAPlanner::PPMAPlanner(const ompl::base::SpaceInformationPtr &si,
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

PPMAPlanner::PPMAPlanner(const ompl::base::SpaceInformationPtr &si,
                         EnvironmentPPMA *environment,
                         bool bSearchForward, double alloc_time, ReplanParams* get_params) : ompl::base::Planner(si, "ppma_planner"),
params(0.0), replan_params(alloc_time), ph_("~") {
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

    reconstructTime = 0;
    totalTime = 0;
    totalPlanTime = 0;
    totalExpands = 0;

    results_file = "BenchmarkMaze.txt";
    output.open(results_file.c_str(),ios::out | ios::app);

    ph_.param<std::string>("sbpl_planner_id",sbpl_planner_id_, "Hstar");

    if(!strcmp(sbpl_planner_id_.c_str(), "Hstar"))
        planner_mode_ = PlannerMode::H_STAR;
    else
        planner_mode_ = PlannerMode::wA_STAR;
}

void PPMAPlanner::InitializeOMPL() {
    specs_.approximateSolutions = true;
    specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("range", this, &PPMAPlanner::setRange,
                                  &PPMAPlanner::getRange,
                                  "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &PPMAPlanner::setGoalBias,
                                  &PPMAPlanner::getGoalBias, "0.:.05:1.");
    setup();
    // for (int i = 0; i < 10; ++i) {
    //  printf("Random num: %f\n", rng_.uniform01());
    // }
}

PPMAPlanner::~PPMAPlanner() {
    freeMemory();
}

void PPMAPlanner::setpdefstartgoal(std::vector<double> start, std::vector<double> goal){
    
    printf("Give me state dimension %d\n", si_->getStateDimension());

    ompl::base::StateSpacePtr space(si_->getStateSpace());
    ompl::base::State* start_state = space->allocState();
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = start[0];
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = start[1];
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = start[2];
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = start[3];
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = start[4];
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = start[5];
    start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = start[6];

    ompl::base::State* goal_state = space->allocState();
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = goal[0];
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = goal[1];
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = goal[2];
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = goal[3];
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = goal[4];
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = goal[5];
    goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = goal[6];

    pdef_->setStartAndGoalStates(start_state, goal_state);
    this->setProblemDefinition(ompl::base::ProblemDefinitionPtr(pdef_));
}

PPMAState *PPMAPlanner::GetState(int id) {
    //if this stateID is out of bounds of our state vector then grow the list
    if (id >= int(states.size())) {
        for (int i = states.size(); i <= id; i++) {
            states.push_back(NULL);
        }
    }

    //if we have never seen this state then create one
    if (states[id] == NULL) {
        states[id] = new PPMAState();
        states[id]->id = id;
        states[id]->replan_number = -1;
    }

    //initialize the state if it hasn't been for this call to replan
    PPMAState *s = states[id];

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

void PPMAPlanner::ExpandState(PPMAState *parent) {
    bool print = false; //parent->id == 285566;

    if (print) {
        printf("expand %d\n", parent->id);
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
        PPMAState *child = GetState(children[i]);
        //if(child->h < 0)
        //    continue;
        //else
            insertLazyList(child, parent, costs[i], isTrueCost[i]);
    }
}

//assumptions:
//state is at the front of the open list
//it's minimum f-value is an underestimate (the edge cost from the parent is a guess and needs to be evaluated properly)
//it hasn't been expanded yet this iteration
void PPMAPlanner::EvaluateState(PPMAState *state) {
    bool print = true; //state->id == 285566;

    if (print) {
        printf("evaluate %d (from %d)\n", state->id, state->best_parent->id);
    }

    PPMAState *parent = state->best_parent;

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
void PPMAPlanner::getNextLazyElement(PPMAState *state) {
    if (state->lazyList.empty()) {
        state->g = INFINITECOST;
        state->best_parent = NULL;
        state->isTrueCost = true;
        return;
    }

    PPMALazyListElement elem = state->lazyList.top();
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

void PPMAPlanner::insertLazyList(PPMAState *state, PPMAState *parent,
                                 int edgeCost, bool isTrueCost) {
    bool print = false; //state->id == 285566 || parent->id == 285566;

    if (parent->id == start_state_id) {
        //printf("Succ has g: %d\n", parent->v + edgeCost);
    }

    if (state->v <= parent->v + edgeCost) {
        return;
    } else if (state->g <= parent->v + edgeCost) {
        //if the best g-value we have is true and better, then the value we had dominates this one and we don't need it
        if (state->isTrueCost) {
            return;
        }

        //insert this guy into the lazy list
        PPMALazyListElement elem(parent, edgeCost, isTrueCost);
        state->lazyList.push(elem);
    } else { //the new guy is the cheapest so far
        //should we save what was the previous best?
        if (!isTrueCost && //the better guy's cost is not for sure
                //state->g < INFINITECOST && //we actually have a previous best (we actually don't need this line because of the next one)
                state->g <
                state->v) { //we're not saving something we already expanded (and is stored in v and expanded_best_parent)
            //we save it by putting it in the lazy list
            PPMALazyListElement elem(state->best_parent, state->g - state->best_parent->v,
                                     state->isTrueCost);
            state->lazyList.push(elem);
            //printf("save the previous best\n");
        }

        //the new guy is the cheapest
        state->g = parent->v + edgeCost;
        state->best_parent = parent;
        state->isTrueCost = isTrueCost;

        if (print) {
            printf("state->id=%d state->g=%d parent->v=%d edgeCost=%d isTrueCost=%d\n",
                   state->id, state->g, parent->v, edgeCost, isTrueCost);
        }

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

void PPMAPlanner::putStateInHeap(PPMAState *state) {
    bool print = false; //state->id == 285566;
    if (state->id == goal_state_id) {
        //printf("putting goal in heap. Printing out goal state\n");
    }

    //we only allow one expansion per search iteration
    //so insert into heap if not closed yet
    if (state->iteration_closed != search_iteration) {
        if (print) {
            printf("put state in open\n");
        }

        CKey key;
        key.key[0] = state->g + int(eps * state->h);

        //if the state is already in the heap, just update its priority
        if (state->heapindex != 0) {
            heap.updateheap(state, key);
        } else { //otherwise add it to the heap
            heap.insertheap(state, key);
        }

        //printf("Added state %d to heap with gval %d and hval %d\n", state->id, state->g, state->h);
        //env_->PrintState(state->id, true);
        if (planner_mode_ == PlannerMode::H_STAR || planner_mode_ == PlannerMode::wA_STAR) {
            // Add to the monolithic tree. TODO(venkat): since we add to the tree even
            // for updateheap, we should update the tree as well by deleting the old
            // motion.
            AddToMonolithicTree(state);
        }
    }
    //if the state has already been expanded once for this iteration
    //then add it to the incons list so we can keep track of states
    //that we know we have better costs for
    else if (!state->in_incons) {
        if (print) {
            printf("put state in incons\n");
        }

        incons.push_back(state);
        state->in_incons = true;
    }
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int PPMAPlanner::ImprovePath() {

    //expand states until done
    int expands = 0;
    CKey min_key = heap.getminkeyheap();

    // OMPL Stuff
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    int highest_minkey = 0;

    bool termination_condition = false;
    bool out_of_time = false;
    while (!termination_condition) {
        //getchar();
        if (planner_mode_ == PlannerMode::wA_STAR) {

            out_of_time = outOfTime();
            termination_condition = heap.emptyheap() ||
                                    min_key.key[0] >= INFINITECOST ||
                                    (goal_state->g <= min_key.key[0] && goal_state->isTrueCost) ||
                                    out_of_time;
        } else if (planner_mode_ == PlannerMode::H_STAR) {
            out_of_time = outOfTime();
            termination_condition = (goal_state->g <= min_key.key[0] && goal_state->isTrueCost) || out_of_time;
        }
        if (termination_condition) {
            //if(!out_of_time){
		//printf("The min key is %d goal g %d goal h %d\n", min_key.key[0], goal_state->g, goal_state->h);
	    //}
	    break;
        }

        if (planner_mode_ == PlannerMode::H_STAR || planner_mode_ == PlannerMode::wA_STAR) {
            // printf("Minkey is %d\n", min_key.key[0]);
            highest_minkey = std::max(highest_minkey, (int)min_key.key[0]);

            //get the state
            PPMAState *state = (PPMAState *)heap.deleteminheap();

            if (state->v == state->g) {
                //printf("ERROR: consistent state is being expanded\n");
                printf("id=%d v=%d g=%d isTrueCost=%d lazyListSize=%lu\n",
                       state->id, state->v, state->g, state->isTrueCost, state->lazyList.size());
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
                //printf("Expanded %d gval %d and with heuristic %d\n", state->id, state->g, state->h);

                if (state->best_parent) {
                    base::State *continuous_state = si_->allocState();
                    env_->GetContState(state->id, continuous_state);
                    base::State *continuous_parent_state = si_->allocState();
                    env_->GetContState(state->best_parent->id, continuous_parent_state);
                    env_->VisualizeContState(continuous_state, continuous_parent_state, true, false);
                }

                if (expands % 100000 == 0) {
                    printf("expands so far=%u\n", expands);
                }
            } else { //otherwise the state needs to be evaluated for its true cost
                EvaluateState(state);
            }
        }


        if (planner_mode_ == PlannerMode::H_STAR || planner_mode_ == PlannerMode::RRT) {
            /* sample random state (with goal biasing) */
            if (goal_s_ && rng_.uniform01() < goalBias_ && goal_s_->canSample()) {
                goal_s_->sampleGoal(rstate);
            } else {
                sampler_->sampleUniform(rstate);
            }

            /* find closest state in the tree */
            Motion *nmotion = monolithic_tree_->nearest(rmotion);
            base::State *dstate = rstate;

            /* find state to add */
            double d = si_->distance(nmotion->state, rstate);

            if (d > maxDistance_) {
                si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d,
                                                  xstate);
                dstate = xstate;
            }

            // double edge_length = si_->distance(nmotion->state, dstate);
            // int num_segments = static_cast<int>(edge_length / 0.0348);
            // double interp_factor = 1.0 / static_cast<double>(num_segments);
            // bool valid = true;
            // base::State *parstate = nmotion->state;
            // base::State *interp_state = si_->allocState();
            // for (double t = 0 ; t <=1; t+=interp_factor) {
            //    si_->getStateSpace()->interpolate(nmotion->state, dstate, t, interp_state);
            //    if (!si_->isValid(interp_state)) {
            //     valid = false;
            //     break;
            //    }
            //    env_->VisualizeContState(interp_state,parstate, false, false);
            //    parstate = interp_state;
            // }
            // si_->freeState(interp_state);

            std::pair<base::State*, double> last_valid;
            last_valid.first = si_->allocState();
            if (si_->checkMotion(nmotion->state, dstate, last_valid)) {

                Motion *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;

                int tree_edge_cost = env_->GetContEdgeCost(nmotion->state, motion->state);
                //printf("The tree edge cost is %d\n", tree_edge_cost);
                //printf("The Nearest motion g is %d\n", nmotion->g);
                motion->g = tree_edge_cost + nmotion->g;

                //printf("The new motion g is %d\n", motion->g);

                monolithic_tree_->add(motion);

                // Goal checking is not necessary for now
                double dist = 0.0;
                bool sat = goal_->isSatisfied(motion->state, &dist);

                //printf("Distance at goal satisfication is %f\n", dist);

                if (sat) {
                    approxdif = dist;
                    solution = motion;
                    //ROS_INFO("RRT found goal!");
                    if (planner_mode_ == PlannerMode::RRT) {
                        lastGoalMotion_ = motion;
                        termination_condition = true;
                    }
                    // break;
                }

                if (dist < approxdif) {
                    approxdif = dist;
                    approxsol = motion;
                }

                // Now add the closest lattice state to the search frontier after
                // computing the appropriate motion cost.

                if (planner_mode_ == PlannerMode::H_STAR) {
                    int nearest_lattice_state_id = -1;
                    base::State *nearest_lattice_state = si_->allocState();
                    env_->GetNearestLatticeState(motion->state, nearest_lattice_state,
                                                 &nearest_lattice_state_id);
                    int snapping_cost = env_->GetContEdgeCost(motion->state,
                                        nearest_lattice_state);

                    //printf("The snapping edge cost is %d\n", snapping_cost);

                    // Add the snap edge to the monolithic tree.
                    /* create a motion */
                    Motion *snap_motion = new Motion(si_);
                    si_->copyState(snap_motion->state, nearest_lattice_state);
                    snap_motion->parent = motion;
                    snap_motion->g = motion->g + snapping_cost;

                    monolithic_tree_->add(snap_motion);
                    std::pair<base::State*, double> last_valid_st;
                    last_valid_st.first = si_->allocState();

                    // printf("Printing Motion state................................\n");
                    // for(int i = 0; i < 7; i++)
                    //     printf("Motion state angle is %f\n", motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
                    // printf("Printing Nearest Lattice state................................\n");
                    // for(int i = 0; i < 7; i++)
                    //     printf("Nearest lattice state angle is %f\n", nearest_lattice_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
                    
                    if (si_->checkMotion(motion->state, nearest_lattice_state, last_valid_st)) {

                        env_->VisualizeContState(nearest_lattice_state,motion->state, false, false);

                        int total_edge_cost = tree_edge_cost + snapping_cost;
                        //printf("the total edge cost is %d\n", total_edge_cost);
                        // TODO(Venkat): I am creating a bogus parent for the
                        // nearest_lattice_state since we are reconstructing the final path
                        // using the OMPL tree. Nevertheless, we should create an
                        // intermediate parent and get IDs for the continuous states from the
                        // environment.
                        // int cont_parent_id = env_->GetContStateID(nmotion->state);
                        PPMAState *cont_child = GetState(nearest_lattice_state_id);
                        //PPMAState *cont_parent = GetState(cont_parent_id);
                        cont_child->g = snap_motion->g;
                        cont_child->isTrueCost = true;
                        putStateInHeap(cont_child);

                        if (nearest_lattice_state_id == goal_state_id) {
                            lastGoalMotion_ = snap_motion;
                            //ROS_INFO("Goal found from snap motion");
                        }

                        //printf("Adding to heap!:g-val: %d h-val: %d min_key: %d\n", cont_child->g, cont_child->h, min_key.key[0]);


                    }
                }
                si_->freeState(last_valid.first);
            }

            //get the min key for the next iteration
        }
        min_key = heap.getminkeyheap();
        //printf("min_key =%d\n",min_key.key[0]);
    }

    search_expands += expands;

    if (out_of_time) {
        return 2;
    }

    if (planner_mode_ == PlannerMode::wA_STAR) {
        if (goal_state->g == INFINITECOST && (heap.emptyheap() ||
                                              min_key.key[0] >= INFINITECOST)) {
            return 0;  //solution does not exists
        }

        printf("wA_STAR search exited with a solution for eps=%.3f\n", eps);

        if (goal_state->g < goal_state->v) {
            goal_state->expanded_best_parent = goal_state->best_parent;
            goal_state->v = goal_state->g;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (lastGoalMotion_ == NULL && planner_mode_ != PlannerMode::wA_STAR) {
        lastGoalMotion_ = approxsol;
        approximate = true;
        //printf("Just an approximate solution.......\n");
    }

    if (lastGoalMotion_ != NULL) {
        // lastGoalMotion_ = solution;
        std::vector<double> soln_angles(7, 0);
        for(int i = 0; i < 7; i++)
        {
            soln_angles[i] = lastGoalMotion_->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
        }
        //env_->printContState(soln_angles);

        solution = lastGoalMotion_;

        /* construct the solution path */
        std::vector<Motion *> mpath;

        while (solution != NULL) {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(si_);

        //printf("Path g-vals");
        for (int i = mpath.size() - 1 ; i >= 0 ; --i) {

            path->append(mpath[i]->state);
            auto se2_state = mpath[i]->state->as<ompl::base::SE2StateSpace::StateType>();
            if(i != 0)
                env_->VisualizeContState(mpath[i]->state,mpath[i-1]->state, false, true);
            else
                env_->VisualizeContState(mpath[i]->state,mpath[i]->state, false, true);
        }
        pdef_->clearSolutionPaths();
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
        //path->print(std::cout);
        solved = true;
        ompl::base::Cost path_cost = path->cost(pdef_->getOptimizationObjective());
        soln_cost = (int)(1000*path_cost.value());
        printf("SOLVED!!! Path has %zu waypoints with solution cost as %d\n", mpath.size(), soln_cost);
    }

    si_->freeState(xstate);

    if (rmotion->state) {
        si_->freeState(rmotion->state);
    }

    delete rmotion;
    OMPL_INFORM("%s: Created %u states", getName().c_str(),
                monolithic_tree_->size());
    // return base::PlannerStatus(solved, approximate);

    return 1;
}

vector<int> PPMAPlanner::GetSearchPath(int &solcost) {
    vector<int> SuccIDV;
    vector<int> CostV;
    vector<bool> isTrueCost;
    vector<int> wholePathIds;

    PPMAState *state;
    PPMAState *final_state;

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
            printf("a state along the path has no parent!\n");
            break;
        }

        if (state->v == INFINITECOST) {
            printf("a state along the path has an infinite g-value!\n");
            printf("inf state = %d\n", state->id);
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
            printf("WARNING: actioncost = %d\n", actioncost);
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

bool PPMAPlanner::outOfTime() {
    //if the user has sent an interrupt signal we stop
    if (interruptFlag) {
        return true;
    }

    //if we are supposed to run until the first solution, then we are never out of time
    if (replan_params.return_first_solution) {
        return false;
    }

    double time_used = double(clock() - TimeStarted) / CLOCKS_PER_SEC;

    if (time_used >= replan_params.max_time) {
        printf("out of max time\n");
    }

    if (use_repair_time && eps_satisfied != INFINITECOST &&
            time_used >= replan_params.repair_time) {
        printf("used all repair time...\n");
    }

    //we are out of time if:
    //we used up the max time limit OR
    //we found some solution and used up the minimum time limit
    return time_used >= replan_params.max_time ||
           (use_repair_time && eps_satisfied != INFINITECOST &&
            time_used >= replan_params.repair_time);
}

void PPMAPlanner::initializeSearch() {
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
    eps = replan_params.initial_eps;
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

    //goal_->print();

    goal_s_ = dynamic_cast<base::GoalSampleableRegion *>(goal_);

    while (const base::State *st = pis_.nextStart()) {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->g = 0;
        monolithic_tree_->add(motion);
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

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
                getName().c_str(), monolithic_tree_->size());

}

bool PPMAPlanner::Search(vector<int> &pathIds, int &PathCost) {
    CKey key;
    TimeStarted = clock();

    initializeSearch();

    //the main loop of PPMA
    while (eps_satisfied > replan_params.final_eps && !outOfTime()) {

        //run weighted A*
        
        chrono::time_point<chrono::monotonic_clock> start, end;
        start = chrono::monotonic_clock::now();
        int before_expands = search_expands;
        
        //ImprovePath returns:
        //1 if the solution is found
        //0 if the solution does not exist
        //2 if it ran out of time
        int ret = ImprovePath();

        end = chrono::monotonic_clock::now();

        if (ret == 1) { //solution found for this iteration
            eps_satisfied = eps;
        }

        int delta_expands = search_expands - before_expands;
        chrono::duration<double> elapsed_seconds = end-start;
        //double delta_time = double(clock() - before_time) / CLOCKS_PER_SEC;
        double delta_time = elapsed_seconds.count();

        //print the bound, expands, and time for that iteration
        printf("bound=%f expands=%d cost=%d time=%.3f\n",
               eps_satisfied, delta_expands, goal_state->g, delta_time);

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
            printf("Solution does not exist\n");
            return false;
        }

        //if we're just supposed to find the first solution
        //or if we ran out of time, we're done
        if (replan_params.return_first_solution || ret == 2) {
            break;
        }

        prepareNextSearchIteration();
    }

    if (planner_mode_ == PlannerMode::wA_STAR || planner_mode_ == PlannerMode::H_STAR) {
        if (goal_state->g == INFINITECOST) {
            totalTime = totalPlanTime;
            printf("wA*/H* could not find a solution (ran out of time)\n");
            return false;
        }

        if (eps_satisfied == INFINITECOST) {
            printf("WARNING: a solution was found but we don't have quality bound for it!\n");
            return false;
        }
    } else {
        if (!lastGoalMotion_) {
            printf("RRT could not find a solution (ran out of time)\n");
        }
    }

    printf("solution found\n");

    // if (planner_mode_ == PlannerMode::wA_STAR) {
    //     clock_t before_reconstruct = clock();
    //     pathIds = GetSearchPath(PathCost);
    //     reconstructTime = double(clock() - before_reconstruct) / CLOCKS_PER_SEC;
    // }
    totalTime = totalPlanTime;

    return true;
}

void PPMAPlanner::prepareNextSearchIteration() {
    //decrease epsilon
    eps -= replan_params.dec_eps;

    if (eps < replan_params.final_eps) {
        eps = replan_params.final_eps;
    }

    //dump the inconsistent states into the open list
    CKey key;

    while (!incons.empty()) {
        PPMAState *s = incons.back();
        incons.pop_back();
        s->in_incons = false;
        key.key[0] = s->g + int(eps * s->h);
        heap.insertheap(s, key);
    }

    //recompute priorities for states in OPEN and reorder it
    for (int i = 1; i <= heap.currentsize; ++i) {
        PPMAState *state = (PPMAState *)heap.heap[i].heapstate;
        heap.heap[i].key.key[0] = state->g + int(eps * state->h);
    }

    heap.makeheap();

    search_iteration++;
}


//-----------------------------Interface function-----------------------------------------------------

void PPMAPlanner::interrupt() {
    interruptFlag = true;
}

int PPMAPlanner::replan(vector<int> *solution_stateIDs_V, ReplanParams p, double &totalT) {
    int solcost;
    return replan(solution_stateIDs_V, p, &solcost, totalT);
}

int PPMAPlanner::replan(int start, int goal, vector<int> *solution_stateIDs_V,
                        ReplanParams p, int *solcost, double &totalT) {
    set_start(start);
    set_goal(goal);
    return replan(solution_stateIDs_V, p, solcost, totalT);
}

int PPMAPlanner::replan(vector<int> *solution_stateIDs_V, ReplanParams p,
                        int *solcost, double &totalT) {
    printf("planner: replan called\n");
    params = p;
    use_repair_time = replan_params.repair_time >= 0;
    interruptFlag = false;
    printf("Goal state id is %d\n", goal_state_id);
    printf("Start state id is %d\n", start_state_id);
    if (goal_state_id < 0) {
        printf("ERROR searching: no goal state set\n");
        return 0;
    }

    if (start_state_id < 0) {
        printf("ERROR searching: no start state set\n");
        return 0;
    }

    //plan
    vector<int> pathIds;
    int PathCost;
    bool solnFound = Search(pathIds, PathCost);
    printf("total expands=%d planning time=%.3f reconstruct path time=%.3f total time=%.3f solution cost=%d\n",
           totalExpands, totalPlanTime, reconstructTime, totalTime, goal_state->g);

    //printf("printing Solnfound\n");
    //printf(solnFound ? "true" : "false");

    output << totalTime << " ";
    output << goal_state->g << " ";
    output << replan_params.initial_eps << "\n";

    totalT = totalTime;
    //copy the solution
    *solution_stateIDs_V = pathIds;
    *solcost = PathCost;

    start_state_id = -1;
    goal_state_id = -1;

    return (int)solnFound;
}

int PPMAPlanner::set_goal(int id) {
    printf("planner: setting goal to %d\n", id);

    if (bforwardsearch) {
        goal_state_id = id;
    } else {
        start_state_id = id;
    }

    return 1;
}

int PPMAPlanner::set_start(int id) {
    printf("planner: setting start to %d\n", id);

    if (bforwardsearch) {
        start_state_id = id;
    } else {
        goal_state_id = id;
    }

    return 1;
}

//--------------------------------------------------------------------------------------------------------
// OMPL stuff
void PPMAPlanner::clear() {
    Planner::clear();
    sampler_.reset();
    freeMemory();

    if (monolithic_tree_) {
        monolithic_tree_->clear();
    }

    lastGoalMotion_ = NULL;
}

void PPMAPlanner::setup() {
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!monolithic_tree_) {
        monolithic_tree_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    }

    monolithic_tree_->setDistanceFunction(boost::bind(
            &PPMAPlanner::distanceFunction, this, _1,
            _2));
}

void PPMAPlanner::freeMemory() {
    if (monolithic_tree_) {
        std::vector<Motion *> motions;
        monolithic_tree_->list(motions);

        for (unsigned int i = 0 ; i < motions.size() ; ++i) {
            if (motions[i]->state) {
                si_->freeState(motions[i]->state);
            }

            delete motions[i];
        }
    }
}

base::PlannerStatus PPMAPlanner::solve(const
                                       base::PlannerTerminationCondition &ptc) {

    bool solved = false;
    bool approximate = false;

    double totalT;

    int bRet = replan(&solution_stateIDs_V,replan_params, totalT);

    printf("done planning\n");
    printf("size of solution=%d\n", (unsigned int)solution_stateIDs_V.size());

    if(bRet) solved = true;

    // checkValidity();
    // base::Goal                 *goal   = pdef_->getGoal().get();
    // base::GoalSampleableRegion *goal_s =
    //   dynamic_cast<base::GoalSampleableRegion *>(goal);

    // while (const base::State *st = pis_.nextStart()) {
    //   Motion *motion = new Motion(si_);
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

    // Motion *solution  = NULL;
    // Motion *approxsol = NULL;
    // double  approxdif = std::numeric_limits<double>::infinity();
    // Motion *rmotion   = new Motion(si_);
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
    //   Motion *nmotion = monolithic_tree_->nearest(rmotion);
    //   base::State *dstate = rstate;

    //   /* find state to add */
    //   double d = si_->distance(nmotion->state, rstate);

    //   if (d > maxDistance_) {
    //     si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d,
    //                                       xstate);
    //     dstate = xstate;
    //   }

    //   if (si_->checkMotion(nmotion->state, dstate)) {
    //     /* create a motion */
    //     Motion *motion = new Motion(si_);
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
    //   std::vector<Motion *> mpath;

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

void PPMAPlanner::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;

    if (monolithic_tree_) {
        monolithic_tree_->list(motions);
    }

    if (lastGoalMotion_) {
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
    }

    for (unsigned int i = 0 ; i < motions.size() ; ++i) {
        if (motions[i]->parent == NULL) {
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        } else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
}

void PPMAPlanner::AddToMonolithicTree(PPMAState *state) {

    // NOTE: Assuming that a state without a backpointer has a continuous parent,
    // and has already been added to the tree.
    // Hence we will omit adding this to the tree at this point.

    if (!state->best_parent) {
        return;
    }

    Motion *parent_motion = new Motion(si_);

    base::State *continuous_parent_state = si_->allocState();
    env_->GetContState(state->best_parent->id, continuous_parent_state);

    // args are ordered by (destination, source), ugh.
    si_->copyState(parent_motion->state, continuous_parent_state);

    /* find closest state in the tree */
    Motion *nearest_motion = monolithic_tree_->nearest(parent_motion);

    /* create the new motion */
    Motion *motion = new Motion(si_);
    base::State *continuous_child_state = si_->allocState();
    env_->GetContState(state->id, continuous_child_state);
    si_->copyState(motion->state, continuous_child_state);
    motion->parent = nearest_motion;
    //printf("Parent g is %d\n", nearest_motion->g);
    int tree_edge_cost = env_->GetContEdgeCost(nearest_motion->state, motion->state);
    //printf("The tree edge cost form addtomonolithic tree is %d\n", tree_edge_cost);
    motion->g = tree_edge_cost + nearest_motion->g;
    //printf("Child g is %d\n", motion->g);

    monolithic_tree_->add(motion);
    //env_->VisualizeContState(motion->state,nearest_motion->state);

    // TODO(venkat): this will break if we have an underspecified goal.
    if (state->id == goal_state_id) {
        lastGoalMotion_ = motion;
    }
}

//---------------------------------------------------------------------------------------------------------

void PPMAPlanner::get_search_stats(vector<PlannerStats> *s) {
    s->clear();
    s->reserve(stats.size());

    for (unsigned int i = 0; i < stats.size(); i++) {
        s->push_back(stats[i]);
    }
}

