#ifndef CEGAR_ORDERSELECTER_H
#define CEGAR_ORDERSELECTER_H

#include "cartesian_heuristic_function.h"
#include "cost_saturation.h"
#include "additive_cartesian_heuristic.h"

#include <ostream>
#include <vector>

namespace cegar {
class AdditiveCartesianHeuristic;

/*
  Decides which new order of the abstractions should be added to the cost partitioning
*/
class OrderSelecter {
    	std::vector<CartesianHeuristicFunction*> *heuristic_functions;
	CostSaturation* cost_saturation;
	utils::RandomNumberGenerator* rng;

public:
    explicit OrderSelecter(CostSaturation* cs, utils::RandomNumberGenerator* rng);

	void set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv);
	std::vector<int> compute_new_order(State state, std::vector<int> max_order);
	std::vector<int> compute_new_order_random(std::vector<int> max_order);
	std::vector<int> compute_new_order_bubble(State state, std::vector<int> max_order, int start_value, AdditiveCartesianHeuristic* heuristic);
	std::vector<int> compute_new_order_swap(State state, std::vector<int> max_order, int start_value, std::vector<bool> toRefine, AdditiveCartesianHeuristic* heuristic);
	std::vector<int> compute_new_order_random_check(State state, std::vector<int> max_order, int start_value, AdditiveCartesianHeuristic* heuristic);

};


}

#endif
