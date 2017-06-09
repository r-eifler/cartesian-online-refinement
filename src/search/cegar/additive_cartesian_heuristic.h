#ifndef CEGAR_ADDITIVE_CARTESIAN_HEURISTIC_H
#define CEGAR_ADDITIVE_CARTESIAN_HEURISTIC_H

#include "../heuristic.h"

#include <vector>

namespace cegar {
class CartesianHeuristicFunction;

/*
  Store CartesianHeuristicFunctions and compute overall heuristic by
  summing all of their values.
*/
class AdditiveCartesianHeuristic : public Heuristic {
	//Online Refinement parameter
	int max_states_online;
	int max_iter;
	int update_h_values;
	
	int online_refined_states = 0;
	
    const std::vector<CartesianHeuristicFunction> heuristic_functions;

    int compute_heuristic(const State &state);

protected:
    virtual int compute_heuristic(const GlobalState &global_state) override;
	virtual std::vector<int> compute_individual_heuristics(const GlobalState &global_state) override;

public:
    explicit AdditiveCartesianHeuristic(const options::Options &opts);
	virtual bool online_Refine(const GlobalState &global_state) override;
	virtual void print_statistics() override;
};
}

#endif
