#ifndef SEARCH_ENGINES_EAGER_SEARCH_H
#define SEARCH_ENGINES_EAGER_SEARCH_H

#include "../search_engine.h"

#include "../open_lists/open_list.h"
#include "../open_lists/open_list_factory.h"

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
	bool refine_online;
    bool use_min_h_value;
	int refinement_threshold;
    int refinement_selector;
    
    //create new openlists
    std::shared_ptr<OpenListFactory> open_list_factory;

    std::unique_ptr<StateOpenList> open_list;
    ScalarEvaluator *f_evaluator;

    std::vector<Heuristic *> heuristics;
    std::vector<Heuristic *> preferred_operator_heuristics;

    std::shared_ptr<PruningMethod> pruning_method;

    int num_nodes_with_improvable_h_value;
    int num_refined_nodes = 0;
    int min_h_value = EvaluationResult::INFTY;
	

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

    void dump_search_space() const;
};
}

#endif
