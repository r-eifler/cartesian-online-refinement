#ifndef SEARCH_ENGINES_REAL_TIME_SEARCH_H
#define SEARCH_ENGINES_REAL_TIME_SEARCH_H

#include "../evaluation_context.h"
#include "../search_engine.h"

#include "../open_lists/open_list.h"
#include "../utils/timer.h"

#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>
#include <list>

namespace options {
class Options;
}

namespace real_time_search {
enum class PreferredUsage {
    PRUNE_BY_PREFERRED,
    RANK_PREFERRED_FIRST
};


enum class LearnStrategy {
	BELLMAN,
	BELLMAN_AND_REFINE,
	REFINE
};

/*
  Enforced hill-climbing with deferred evaluation.

  TODO: We should test if this lazy implementation really has any benefits over
  an eager one. We hypothesize that both versions need to evaluate and store
  the same states anyways.
*/
class RealTimeSearch : public SearchEngine {
    std::unique_ptr<StateOpenList> open_list;
    std::unique_ptr<StateOpenList> open_list_learn;

    Heuristic *heuristic;
    std::vector<Heuristic *> preferred_operator_heuristics;
    std::set<Heuristic *> heuristics;
    bool use_preferred;
    PreferredUsage preferred_usage;
	LearnStrategy learn_strategy;

    EvaluationContext current_eval_context;
    int current_phase_start_g;

	//real time parameter
	double time_unit;
	double lookahead_fraction;
	bool refine_commited_state;
	bool use_refine_time_bound;
	bool refine_base;
	bool refine_to_frontier;
	utils::Timer step_timer;
	std::vector<GlobalState> next_expanded_state;
	

    // Statistics
    std::map<int, std::pair<int, int>> d_counts;
    int num_ehc_phases;
    int last_num_expanded;
	float game_time = 0;
	float lookahead_time = 0;

	//Collect plan during execution
	std::vector<const GlobalOperator*> real_time_plan;
	std::list<StateID> expand_states;

    void reach_state(
        const GlobalState &parent, const GlobalOperator &op,
        const GlobalState &state);
	bool refine_root_to_frontier(double time_bound);
	bool bellman_dijkstra_backup(double time_bound,const GlobalState &s);
	bool refine_heuristic(double time_bound);

	bool compute_next_real_time_step(const GlobalState &s, bool solution_found);
	void reset_search_and_execute_next_step(const GlobalState &s);
	bool update_heuristic(const GlobalState &s);

    SearchStatus search();

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit RealTimeSearch(const options::Options &opts);
    virtual ~RealTimeSearch() override;

    virtual void print_statistics() const override;
};
}

#endif
