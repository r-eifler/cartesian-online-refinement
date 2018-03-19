#ifndef CEGAR_ONLINEREFINEMENT_H
#define CEGAR_ONLINEREFINEMENT_H

#include "cartesian_heuristic_function.h"
#include "cost_saturation.h"

#include <ostream>
#include <vector>

namespace cegar {

/*
  Decides which new order of the abstractions should be added to the cost partitioning
*/
class OnlineRefinement {
    	std::vector<CartesianHeuristicFunction*> *heuristic_functions;
	CostSaturation* cost_saturation;
	utils::RandomNumberGenerator* rng;
	

	int max_states_online;
	bool use_usefull_split = false;
	int online_refined_states = 0;

	utils::Timer timer;

public:
    explicit OnlineRefinement(CostSaturation* cs, utils::RandomNumberGenerator* rng, int mso, bool use_us);

	void set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv);
	bool refine(State state, std::vector<bool> toRefine, std::vector<State> frontier_nodes);

	void print_statistics();

};


}

#endif
