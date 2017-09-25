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
	
enum class Strategy {
    ORDER_REFINE,
	REFINE_ORDER,
	ONLY_ORDER,
	ONLY_REFINE
};
	


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
	bool use_merge;	
	bool prove_bellman;
	int threshold = 0;
	bool local_minimum = false;
	
	Strategy strategy = Strategy::ORDER_REFINE;
	
	int sat_bellman = 0;
	int not_sat_bellman = 0;
	int refine_steps_total = 0;
	int refined_states_total = 0;
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
	// returns a vector which contains the heuristic value of each abstraction for the given state
	virtual std::vector<int> compute_individual_heuristics(const GlobalState &global_state) override;
	
	bool refine(State state, int* current_max_h, std::vector<bool> &toRefine);
	bool reorder(State state, int* current_max_h, std::vector<bool> &toRefine);
	
	//bool prove_bellman_sum(State state, std::vector<std::pair<GlobalState, int>> succStates, std::vector<bool> *toRefine, int* current_h);
	bool prove_bellman_individual(GlobalState global_state, std::vector<std::pair<GlobalState, int>> succStates, std::vector<bool> *toRefine, int* current_h, bool* conflict);
	bool prove_bellman_sum(GlobalState global_state, std::vector<std::pair<GlobalState, int>> succStates, int* current_h);

public:
    explicit AdditiveCartesianHeuristic(const options::Options &opts);
	
	virtual bool online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates) override;
	
	std::vector<int> compute_original_individual_heuristics(State state);
	
	std::vector<int> compute_individual_heuristics_of_order(const GlobalState &global_state, int order);
	std::vector<int> compute_individual_heuristics_of_order(const State state, int order);
	
	virtual void print_statistics() override;
	void print_order();
	//computes the heuristic based on the last scp order which has been used
	int compute_current_order_heuristic(const State state);
};
}

#endif
