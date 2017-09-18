#ifndef SEARCH_ENGINES_DFS_PRUNING_H
#define SEARCH_ENGINES_DFS_PRUNING_H

#include "../search_engine.h"

#include "../open_lists/open_list.h"
#include "../open_lists/open_list_factory.h"
#include "../utils/timer.h"

#include "../cegar/additive_cartesian_heuristic.h"

#include <memory>
#include <vector>

class GlobalOperator;
class Heuristic;
class PruningMethod;
class ScalarEvaluator;

namespace options {
class Options;
}



namespace dfs_pruning {
	
enum class CheckStrategy {
	BOUND,
	BELLMAN,
	BOUND_AND_BELLMAN
};	
	
class DFSPruning : public SearchEngine {
    const bool reopen_closed_nodes;
    const bool use_multi_path_dependence;
	
	//Online Refinement ops
	//flag use online refinement (default true)
	bool refine_online;
	// only every refinement_selector state is tried to be refined
    int refinement_selector;
	// only every refinement_time seconds a state is tried to be refined
	double refinement_time;
	int collect_states;
	CheckStrategy check_strategy = CheckStrategy::BOUND;
	
	bool need_to_refine = false;
	std::vector<std::pair<GlobalState, int>> states_to_refine;

    std::unique_ptr<StateOpenList> open_list;
    ScalarEvaluator *f_evaluator;

    std::vector<Heuristic *> heuristics;
    std::vector<Heuristic *> preferred_operator_heuristics;
	
	ScalarEvaluator* pruning_heuristic;

    std::shared_ptr<PruningMethod> pruning_method;
	
	int upper_bound = EvaluationResult::INFTY;
	GlobalState* current_goal_state = NULL;
	
	std::vector<GlobalState> current_solution;
	std::vector<GlobalState> current_path;

	//number of nodes which have have been refined 
    int num_refined_nodes = 0;
	
	//TIMER
	utils::Timer total_refine_timer;
	utils::Timer print_timer;
	utils::Timer refine_timer;
	utils::Timer select_timer;
	
	int num_pruned_states = 0;
	int num_refined_states = 0;
	int openlist_size = 0;
	int update_steps = 0;
	
    std::pair<SearchNode, int> fetch_next_node();
	void refine(bool backtracked, int backtrack_depth, GlobalState expandedState);
	bool refine_bound(GlobalState backtrackedState, GlobalState expandedState);
	bool refine_bellman(GlobalState expandedState);
    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(const SearchNode &node);
    void reward_progress();
    void print_checkpoint_line(int g) const;

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit DFSPruning(const options::Options &opts);
    virtual ~DFSPruning() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};
}

#endif
