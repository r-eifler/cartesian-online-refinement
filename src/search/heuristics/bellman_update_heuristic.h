#ifndef HEURISTICS_BLIND_SEARCH_HEURISTIC_H
#define HEURISTICS_BLIND_SEARCH_HEURISTIC_H

#include "../heuristic.h"
#include <map>

namespace bellman_update_heuristic {
class BellmanUpdateHeuristic : public Heuristic {

    int min_operator_cost;
	std::map<int, int> h_values;
	ScalarEvaluator* base_heuristic;

protected:
    virtual int compute_heuristic(const GlobalState &global_state);
	virtual bool online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates);
public:
    BellmanUpdateHeuristic(const options::Options &options);
    ~BellmanUpdateHeuristic();
};
}

#endif
