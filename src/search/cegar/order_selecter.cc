#include "order_selecter.h"

#include <sstream>
#include <iostream>
#include <chrono>

using namespace std;

namespace cegar {
OrderSelecter::OrderSelecter(CostSaturation* cs, utils::RandomNumberGenerator* r) :
	cost_saturation(cs),
	rng(r){

}

void OrderSelecter::set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv){
	heuristic_functions = fv;
}

std::vector<int> OrderSelecter::compute_new_order_random(std::vector<int> max_order){
	vector<int> current_order(max_order);
	//Shuffel new_order
    	shuffle(current_order.begin(), current_order.end(), default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
	return current_order;
}

std::vector<int> OrderSelecter::compute_new_order(State state, std::vector<int> max_order){
		
	vector<int> current_order(max_order);

	//swap the first abstraction in the order with any other abstraction
	
	int new_pos = (*rng)(current_order.size() - 1);
	new_pos = new_pos == 0 ? 1 : new_pos;
    	iter_swap(current_order.begin(), current_order.begin() + (new_pos)); 
	

	//push all abstractions which are satisfied by the state to the end
	vector<int> new_order;
    	vector<int> sat_abs;
    	for(int pos : current_order){
		CartesianHeuristicFunction *function = (*heuristic_functions)[pos];
		if(function->satisfies_goal(state)){
			sat_abs.push_back(pos);
	   	}
	   	else{
	       		new_order.push_back(pos);
	   	}	
    	}
    	
    	new_order.insert( new_order.end(), sat_abs.begin(), sat_abs.end() );


	return new_order;
}

std::vector<int> OrderSelecter::compute_new_order_bubble(State state, std::vector<int> max_order, int start_value, AdditiveCartesianHeuristic* heuristic){

	vector<int> current_order(max_order);

	//swap the ith abstraction in the order with any other abstraction behinde it
	for(size_t p = 0; p < current_order.size() - 1; p++){
		int new_pos = (*rng)(current_order.size() - 1 - p);
		new_pos = new_pos == 0 ? 1 : new_pos;
		new_pos += p;
		iter_swap(current_order.begin(), current_order.begin() + (new_pos)); 
		cost_saturation->recompute_cost_partitioning(current_order);
		int new_value = heuristic->compute_current_order_heuristic(state);
		//cout << "old: " << start_value << " new: " << new_value << endl;
		if(new_value < start_value){			
			iter_swap(current_order.begin(), current_order.begin() + (new_pos)); 
			continue;
		}
		start_value = new_value;
	}
	
	
	return current_order;
}

std::vector<int> OrderSelecter::compute_new_order_swap(State state, std::vector<int> max_order, int start_value, std::vector<bool> toRefine, AdditiveCartesianHeuristic* heuristic){
	vector<int> current_order(max_order);
	int new_value = 0;
	//swap the ith abstraction in the order with any other abstraction behinde it
	for(size_t p = 1; p < toRefine.size(); p++){
		//if(toRefine[p] && ! toRefine[p-1]){
		if(toRefine[p] && toRefine[p-1]){
			iter_swap(current_order.begin() + p, current_order.begin() + p - 1);
			cost_saturation->recompute_cost_partitioning(current_order);
			new_value = heuristic->compute_current_order_heuristic(state);
			//cout << "old: " << start_value << " new: " << new_value << endl;
			if(new_value < start_value){			
				iter_swap(current_order.begin() + p, current_order.begin() + p - 1); 
				continue;
			}
			start_value = new_value;
		}
	}
	return current_order; 	
}

std::vector<int> OrderSelecter::compute_new_order_random_check(State state, std::vector<int> max_order, int start_value, AdditiveCartesianHeuristic* heuristic){

	vector<int> current_order(max_order);
	int new_value = 0;
	//swap the ith abstraction in the order with any other abstraction behinde it
	int tries = 0;
	do{
		shuffle(current_order.begin(), current_order.end(), default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
		cost_saturation->recompute_cost_partitioning(current_order);
		new_value = heuristic->compute_current_order_heuristic(state);
		//cout << "old: " << start_value << " new: " << new_value << endl;
		tries++;
	}while(new_value <= start_value && tries < 10);
	
	
	return current_order;
}


}
