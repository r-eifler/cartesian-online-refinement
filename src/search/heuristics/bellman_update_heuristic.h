#ifndef HEURISTICS_BLIND_SEARCH_HEURISTIC_H
#define HEURISTICS_BLIND_SEARCH_HEURISTIC_H

#include "../heuristic.h"
#include <map>

namespace bellman_update_heuristic {
class BellmanUpdateHeuristic : public Heuristic {

    int min_operator_cost;
	std::unordered_map<int, int> h_values;
	ScalarEvaluator* base_heuristic;
	bool refine_abstractions;

protected:
    virtual int compute_heuristic(const GlobalState &global_state);
	virtual bool online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates, std::vector<GlobalState> new_goals, double time_bound);
	virtual bool online_Refine_base(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates, std::vector<GlobalState> new_goals, double time_bound);
	virtual void update(const GlobalState &global_state, int h);
public:
    BellmanUpdateHeuristic(const options::Options &options);
    ~BellmanUpdateHeuristic();
};
}

#endif
