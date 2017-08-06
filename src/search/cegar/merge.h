#ifndef CEGAR_MERGE_H
#define CEGAR_MERGE_H

#include "cartesian_heuristic_function.h"
#include "cost_saturation.h"

#include <ostream>
#include <vector>

namespace cegar {
	
enum class MergeStrategy {
    COMPATIBLE_PLANS,
	SMALLEST
};

/*
  Decides which abstraction should be merged and merges them 
*/
class Merge {
    	std::vector<CartesianHeuristicFunction*> *heuristic_functions;
	CostSaturation* cost_saturation;
	utils::RandomNumberGenerator* rng;

	int max_merge_size = 100000;
	int merged_abs = 0;
	int max_size_reached = false;
	MergeStrategy merge_strategy = MergeStrategy::COMPATIBLE_PLANS;

	utils::Timer merge_timer;
protected:
	//selects the two smallest abstractions
	int select_smallest_abstractions(CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);
	//selects the two smallest abstractions which need to be refined
	int select_smallest_abstractions_and_refine(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);
	
	//selects the two smallest abstractions which have a common scp computed on the original cost
	int select_common_saturation(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);
	bool intersect(std::vector<int> *cp1, std::vector<int> *cp2);
	
	//selects the two smallest abstractions which habe a not compatible plan
	int select_compatible_plan(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2);

public:
    explicit Merge(CostSaturation* cs, utils::RandomNumberGenerator* rng, MergeStrategy ms);

	void set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv);
	bool merge(std::vector<bool> toRefine);

	void print_statistics();

};


}

#endif
