#ifndef CEGAR_MERGE_H
#define CEGAR_MERGE_H

#include "cartesian_heuristic_function.h"
#include "cost_saturation.h"

#include <ostream>
#include <vector>

namespace cegar {

/*
  Decides which abstraction should be merged and merges them 
*/
class Merge {
    	std::vector<CartesianHeuristicFunction*> *heuristic_functions;
	CostSaturation* cost_saturation;
	utils::RandomNumberGenerator* rng;

	int max_merge_size = 10000;
	int merged_abs = 0;

	utils::Timer merge_timer;
protected:
	int select_smallest_abstractions(CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);

public:
    	explicit Merge(CostSaturation* cs, utils::RandomNumberGenerator* rng);

	void set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv);
	bool merge();

	void print_statistics();

};


}

#endif
