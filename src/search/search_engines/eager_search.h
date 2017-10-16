#ifndef SEARCH_ENGINES_EAGER_SEARCH_H
#define SEARCH_ENGINES_EAGER_SEARCH_H

#include "../search_engine.h"

#include "../open_lists/open_list.h"
#include "../open_lists/open_list_factory.h"
#include "../utils/timer.h"

#include <memory>
#include <vector>

class GlobalOperator;
class Heuristic;
class PruningMethod;
class ScalarEvaluator;

namespace options {
class Options;
}

namespace eager_search {
class EagerSearch : public SearchEngine {
    const bool reopen_closed_nodes;
    const bool use_multi_path_dependence;
	
	//Online Refinement ops
	//flag use online refinement (default true)
	bool refine_online;
	// only every refinement_time seconds a state is tried to be refined
	double refinement_waiting = 0;
	bool wait_time = false;
	int collect_states;
	
	bool need_to_refine = false;
	std::vector<std::pair<GlobalState, int>> states_to_refine;
    
    //create new openlists
    std::shared_ptr<OpenListFactory> open_list_factory;

    std::unique_ptr<StateOpenList> open_list;
    ScalarEvaluator *f_evaluator;

    std::vector<Heuristic *> heuristics;
    std::vector<Heuristic *> preferred_operator_heuristics;

    std::shared_ptr<PruningMethod> pruning_method;

	//number of nodes which have have been refined 
    int num_refined_nodes = 0;
	
	//TIMER
	utils::Timer open_list_timer;
	utils::Timer total_refine_timer;
	utils::Timer print_timer;
	utils::Timer refine_timer;
	
	// number of stets whose heuristic value increased since the last evaluation
	// need to be put back into the open list
	int num_reeval_states = 0;

    std::pair<SearchNode, bool> fetch_next_node();
    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(const SearchNode &node);
    void reward_progress();
    void print_checkpoint_line(int g) const;

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit EagerSearch(const options::Options &opts);
    virtual ~EagerSearch() = default;

    virtual void print_statistics() const override;
	virtual void reset_search() override;

    void dump_search_space() const;
};
}

#endif
