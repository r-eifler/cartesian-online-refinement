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

	int max_merge_size = 5000;
	int merged_abs = 0;

	utils::Timer merge_timer;
protected:
	int select_smallest_abstractions(CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);
	int select_smallest_abstractions_and_refine(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);
	
	int select_common_saturation(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);
	bool intersect(std::vector<int> *cp1, std::vector<int> *cp2);
	int select_compatible_plan(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);

public:
    	explicit Merge(CostSaturation* cs, utils::RandomNumberGenerator* rng);

	void set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv);
	bool merge(std::vector<bool> toRefine);

	void print_statistics();

};


}

#endif
