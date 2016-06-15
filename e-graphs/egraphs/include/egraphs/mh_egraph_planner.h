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
#ifndef _MH_EGRAPH_PLANNER_H_
#define _MH_EGRAPH_PLANNER_H_

#include <sbpl/headers.h>
#include <queue>
#include <egraphs/egraphManager.h>
#include <egraphs/planner_state.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <egraphs/egraph_planner.h>

//class LazyAEGListElement;

enum MetaSearchType{
  ROUND_ROBIN,
  META_A_STAR,
  DTS
};

enum PlannerType{
  IMHA,
  SMHA
};

/*
class EGraphReplanParams : public ReplanParams{
  public:
    EGraphReplanParams(double time):ReplanParams(time) {
      epsE = 10.0;
      final_epsE = 1.0;
      dec_epsE = 1.0;
      feedback_path = true;
      use_egraph = true;
      use_lazy_validation = true;
    };
    double epsE;
    double final_epsE;
    double dec_epsE;
    bool feedback_path;
    bool use_egraph;
    bool update_stats;
    bool use_lazy_validation;
    bool validate_during_planning;
};
*/

template <typename HeuristicType>
class MHEGraphPlanner : public SBPLPlanner{
    typedef EGraphManager<HeuristicType>* EGraphManagerPtr;

    public:
        void interrupt();

        virtual int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V){
            printf("Not supported. Use ReplanParams");
            return -1;
        };
        virtual int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V, int* solcost){
            printf("Not supported. Use ReplanParams");
            return -1;
        };

        virtual int replan(int start, vector<int>* solution_stateIDs_V, 
                           EGraphReplanParams params, int* solcost);
        virtual int replan(std::vector<int>* solution_stateIDs_V, EGraphReplanParams params);
        virtual int replan(std::vector<int>* solution_stateIDs_V, EGraphReplanParams params, int* solcost);

        virtual int set_goal(int goal_stateID){ROS_WARN("set_goal is not used. we assume the goal conditions have been set in the environment and use EGraphable::isGoal");return 1;};
        virtual int set_goal();
        virtual int set_start(int start_stateID);

        virtual void costs_changed(StateChangeQuery const & stateChange){return;};
        virtual void costs_changed(){return;};

        virtual int force_planning_from_scratch(){return 1;};
        virtual int force_planning_from_scratch_and_free_memory(){return 1;};

        virtual int set_search_mode(bool bSearchUntilFirstSolution){
            printf("Not supported. Use ReplanParams");
            return -1;
        };

        virtual void set_initialsolution_eps(double initialsolution_eps){
            printf("Not supported. Use ReplanParams");
        };

        MHEGraphPlanner(DiscreteSpaceInformation* environment, int num_heur, 
                       EGraphManagerPtr egraph_mgr);
        ~MHEGraphPlanner();

        map<string,double> getStats(){return stat_map_;};
        virtual void get_search_stats(vector<PlannerStats>* s);
        void feedback_last_path();
        void setLazyValidation(bool b){ params.use_lazy_validation = b; };
        void setValidateDuringPlanning(bool b){ params.validate_during_planning = b; };

    protected:
        //data structures (open and incons lists)
        std::vector<CHeap> heaps;
        vector<vector<LazyAEGState*> > incons;
        vector<vector<LazyAEGState*> > states;

        EGraphReplanParams params;
        EGraphManagerPtr egraph_mgr_;

        bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
        LazyAEGState goal_state;
        LazyAEGState* start_state;
        //int goal_state_id;
        int start_state_id;

        //mha params
        int num_heuristics;
        PlannerType planner_type;
        MetaSearchType meta_search_type;
        std::vector<int> queue_expands;
        std::vector<int> queue_best_h;
        std::vector<int> meta_queue_best_h;
        int max_heur_dec;
        std::vector<double> alpha;
        std::vector<double> beta;
        double betaC;
        const gsl_rng_type* gsl_rand_T;
        gsl_rng* gsl_rand;

        //search member variables
        double eps;
        double eps_satisfied;
        int search_expands;
        //clock_t TimeStarted;
        double TimeStarted;
        double metaTime;
        short unsigned int search_iteration;
        short unsigned int replan_number;
        bool use_repair_time;

        //stats
        map<string,double> stat_map_;
        vector<PlannerStats> stats;
        unsigned int totalExpands;
        double totalPlanTime;
        double reconstructTime;
        double feedbackPathTime;
        double heuristicSetGoalTime;
        double succsClock;
        double shortcutClock;
        double heuristicClock;
        //clock_t succsClock;
        //clock_t shortcutClock;
        //clock_t heuristicClock;

        int evaluated_snaps;

        bool interruptFlag;

        bool reconstructSuccs(LazyAEGState* state, LazyAEGState*& next_state, 
                              vector<int>* wholePathIds, vector<int>* costs);

        virtual LazyAEGState* GetState(int q_id, int id);
        virtual void ExpandState(int q_id, LazyAEGState* parent);
        virtual void EvaluateState(int q_id, LazyAEGState* parent);
        void getNextLazyElement(int q_id, LazyAEGState* state);
        void insertLazyList(int q_id, LazyAEGState* state, LazyAEGState* parent, int edgeCost, bool isTrueCost, EdgeType edgeType, int snap_midpoint);
        void putStateInHeap(int q_id, LazyAEGState* state);
        bool updateGoal(LazyAEGState* state);

        virtual int ImprovePath();
        void checkHeaps(string msg);

        virtual vector<int> GetSearchPath(int& solcost);

        virtual bool outOfTime();
        virtual void initializeSearch();
        virtual void prepareNextSearchIteration();
        virtual bool Search(vector<int>& pathIds, int & PathCost);

        virtual int GetBestHeuristicID();

        int thread_heur_id_;
        HeuristicType thread_heur_coord_;
        void getAllHeuristics(int state_id);
        void initializeHeuristics();
        boost::mutex heuristic_mutex_;
        int num_threads_;
        int work_id_;
        bool planner_ok_;
        int sleeping_;
        int reported_for_duty_;
        bool heuristics_initialized_;
        boost::condition_variable heuristic_cond_;
        boost::condition_variable main_thread_cond_;
        std::vector<boost::thread*> threads_;
};

#include<egraphs/../../src/mh_egraph_planner.cpp>

#endif
