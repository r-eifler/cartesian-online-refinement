#ifndef CEGAR_ADDITIVE_CARTESIAN_HEURISTIC_H
#define CEGAR_ADDITIVE_CARTESIAN_HEURISTIC_H

#include "../heuristic.h"
#include "cost_saturation.h"

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
	utils::Timer cost_timer;
	utils::Timer refine_timer;
	utils::Timer prove_timer;
	utils::Timer merge_timer;
	utils::Timer print_timer;
    utils::Timer update_timer;
	
	int improved_order = 0;
	int decreased_order = 0;
	int improved_refine = 0;
	
	int current_order = 0;
	
	CostSaturation* cost_saturation;
    std::vector<CartesianHeuristicFunction*> heuristic_functions;

    int compute_heuristic(const State &state);
	std::vector<CartesianHeuristicFunction*> generate_heuristic_functions(const options::Options &opts);

protected:
    virtual int compute_heuristic(const GlobalState &global_state) override;
	virtual std::vector<int> compute_individual_heuristics(const GlobalState &global_state) override;

public:
    explicit AdditiveCartesianHeuristic(const options::Options &opts);
	virtual bool online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates, int* new_order) override;
	virtual void print_statistics() override;
	void print_order();
	virtual void change_to_order(int id) override;
};
}

#endif
