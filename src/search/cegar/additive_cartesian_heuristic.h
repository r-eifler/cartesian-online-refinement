#ifndef CEGAR_ADDITIVE_CARTESIAN_HEURISTIC_H
#define CEGAR_ADDITIVE_CARTESIAN_HEURISTIC_H

#include "../heuristic.h"
#include "cost_saturation.h"
#include "order_selecter.h"
#include "online_refinement.h"
#include "merge.h"

#include <random>
#include <algorithm>

#include <vector>

namespace cegar {
class OrderSelecter;	
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
	bool use_all_goals;
	
	int online_refined_states = 0; 	
	utils::Timer cost_timer;
	utils::Timer refine_timer;
	utils::Timer prove_timer;
	utils::Timer merge_timer;
	utils::Timer print_timer;
    utils::Timer update_timer;
	utils::Timer values_timer;
	
	utils::Timer delete_timer;
	bool deleted = false;
	
	int improved_order = 0;
	int decreased_order = 0;
	int improved_refine = 0;
	int improved_merge = 0;
	int refinement_pathology = 0;
	int merged_abstractions = 0;
	
	int current_order = 0;
	int number_of_orders = 1;
	std::vector<int> usefullnes_of_order;
	std::vector<int> lifetime_of_order;
	utils::RandomNumberGenerator* rng_order;
	int n_calls = 0;
	std::vector<int> usefullnes_of_abstraction;
	
	
	CostSaturation* cost_saturation;	
    std::vector<CartesianHeuristicFunction*> heuristic_functions;
	OrderSelecter* orderSelecter;
	OnlineRefinement onlineRefinement;
	Merge merge;

    int compute_heuristic(const State &state);
	std::vector<CartesianHeuristicFunction*> generate_heuristic_functions(const options::Options &opts);

protected:
    virtual int compute_heuristic(const GlobalState &global_state) override;
	virtual std::vector<int> compute_individual_heuristics(const GlobalState &global_state) override;

public:
    explicit AdditiveCartesianHeuristic(const options::Options &opts);
	
	std::vector<int> compute_individual_heuristics_of_order(const GlobalState &global_state, int order);
	std::vector<int> compute_individual_heuristics_of_order(const State state, int order);
	virtual bool online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates) override;
	virtual void print_statistics() override;
	void print_order();
	virtual void change_to_order(int id) override;
	int compute_current_order_heuristic(const State state);
};
}

#endif
