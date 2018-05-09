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

/*
  Enforced hill-climbing with deferred evaluation.

  TODO: We should test if this lazy implementation really has any benefits over
  an eager one. We hypothesize that both versions need to evaluate and store
  the same states anyways.
*/
class RealTimeSearch : public SearchEngine {
    std::unique_ptr<StateOpenList> open_list;

    Heuristic *heuristic;
    std::vector<Heuristic *> preferred_operator_heuristics;
    std::set<Heuristic *> heuristics;
    bool use_preferred;
    PreferredUsage preferred_usage;

    EvaluationContext current_eval_context;
    int current_phase_start_g;

	//real time parameter
	double time_unit;
	double lookahead_fraction;
	bool use_refine_time_bound;
	bool refine_base;
	bool refine_to_frontier;
	utils::Timer step_timer;
	

    // Statistics
    std::map<int, std::pair<int, int>> d_counts;
    int num_ehc_phases;
    int last_num_expanded;
	float game_time = 0;

	//Collect plan during execution
	std::vector<const GlobalOperator*> real_time_plan;
	std::list<StateID> expand_states;

    void reach_state(
        const GlobalState &parent, const GlobalOperator &op,
        const GlobalState &state);
	SearchStatus compute_next_real_time_step(GlobalState s, bool solution_found, int min_h);
	bool refine_valley(GlobalState next_expanded_state, int min_h);
	bool refine_root_to_frontier(double time_bound);
	bool refine_expanded(double time_bound);
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
