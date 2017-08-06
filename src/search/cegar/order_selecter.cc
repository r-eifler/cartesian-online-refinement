#include "order_selecter.h"

#include <sstream>
#include <iostream>
#include <chrono>

using namespace std;

namespace cegar {
OrderSelecter::OrderSelecter(CostOrder co, CostSaturation* cs, utils::RandomNumberGenerator* r) :
	orderType(co),
	cost_saturation(cs),
	rng(r){

	timer.stop();
}

void OrderSelecter::set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv){
	heuristic_functions = fv;
}

	
std::vector<int> OrderSelecter::compute(State state, int maxOrderId, std::vector<int> max_order, int start_value, std::vector<bool> toRefine, AdditiveCartesianHeuristic* heuristic){
	
	switch (orderType){
		case CostOrder::SHUFFLE : 
			return compute_new_order_random(max_order);
		case CostOrder::SWAP_FIRST_SAT :
			return compute_new_order(state, max_order);
		case CostOrder::BUBBLE :
			return compute_new_order_bubble(state, max_order, start_value, heuristic);
		case CostOrder::SWAP_ALL_CHECK :
			return compute_new_order_swap(state, max_order, start_value, toRefine, heuristic);
		case CostOrder::SHUFFLE_CHECK :
			return compute_new_order_random_check(state, max_order, start_value, heuristic);
		case CostOrder::ORDER_ASC :
			return compute_order_heuristic_rev(state, maxOrderId, heuristic);
		case CostOrder::ORDER_DESC :	
			return compute_order_heuristic(state, maxOrderId, heuristic);
		case CostOrder::ORDER_ORG_ASC:
			return compute_order_org_heuristic_rev(state, heuristic);
		case CostOrder::ORDER_ORG_DESC:
			return compute_order_org_heuristic(state, heuristic);
	}
	
	return max_order;
}
	
std::vector<int> OrderSelecter::compute_new_order_random(std::vector<int> max_order){
	timer.resume();
	vector<int> current_order(max_order);
	//Shuffel new_order
    	shuffle(current_order.begin(), current_order.end(), default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
	
	timer.stop();
	return current_order;
}
	


std::vector<int> OrderSelecter::compute_new_order(State state, std::vector<int> max_order){
	timer.resume();
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

	timer.stop();
	return new_order;
}

std::vector<int> OrderSelecter::compute_new_order_bubble(State state, std::vector<int> max_order, int start_value, AdditiveCartesianHeuristic* heuristic){
	timer.resume();
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
	
	timer.stop();
	return current_order;
}

std::vector<int> OrderSelecter::compute_new_order_swap(State state, std::vector<int> max_order, int start_value, std::vector<bool> toRefine, AdditiveCartesianHeuristic* heuristic){
	timer.resume();
	vector<int> current_order(max_order);
	int new_value = 0;
	//swap the ith abstraction in the order with the i+1th abstraction
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
	timer.stop();
	return current_order; 	
}

std::vector<int> OrderSelecter::compute_new_order_random_check(State state, std::vector<int> max_order, int start_value, AdditiveCartesianHeuristic* heuristic){
	timer.resume();
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
	
	timer.stop();
	return current_order;
}
	
std::vector<int> OrderSelecter::compute_order_heuristic(State state, int maxOrderId, AdditiveCartesianHeuristic* heuristic){
	timer.resume();
	vector<int> current_values = heuristic->compute_individual_heuristics_of_order(state, maxOrderId);
	/*
	cout << "Original Values: ";
	for(int v : current_values){
		cout << v << " ";	
	}
	cout << endl;
	*/
	//Sort the functions according to the highest value in curretn_values
	vector<pair<int,int>> valpos;
	for(size_t i = 0; i < current_values.size(); i++){
		valpos.push_back(make_pair(i, current_values[i]));
	}
	vector<int> new_order;
	while(valpos.size() > 0){
		/*
		cout << "ValPos: ";
		for(pair<int,int> v : valpos){
			cout << "(" << v.first << "|" << v.second  << ") ";	
		}
		cout << endl;
		*/
		pair<int,int> max = valpos[0];
		int pos = 0;
		for(size_t i = 0; i < valpos.size(); i++){
			if(valpos[i].second > max.second){
				max = valpos[i];
				pos = i;
			}			
		}
		//cout << "Max: (" << max.first << "|" << max.second  << ") Pos: " << pos << endl; 
		valpos.erase(valpos.begin() + pos);
		new_order.push_back(max.first);
	}
	/*
	cout << "New Order: ";
	for(int v : new_order){
		cout << v << " ";	
	}
	cout << endl;
	cout << "-----------------------------------" << endl;
	*/
	timer.stop();
	return new_order;
}
	
std::vector<int> OrderSelecter::compute_order_heuristic_rev(State state, int maxOrderId, AdditiveCartesianHeuristic* heuristic){
	timer.resume();
	vector<int> ordered_values = compute_order_heuristic(state, maxOrderId, heuristic);
	vector<int> rev_order;
	for(int i = ordered_values.size() - 1; i >= 0; i--){
		rev_order.push_back(ordered_values[i]);	
	}
	vector<int> new_order;
    vector<int> sat_abs;
	for(int pos : rev_order){
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
	
	
std::vector<int> OrderSelecter::compute_order_org_heuristic(State state, AdditiveCartesianHeuristic* heuristic){
	timer.resume();
	vector<int> current_values = heuristic->compute_original_individual_heuristics(state);
	
	//Sort the functions according to the highest value in curretn_values
	vector<pair<int,int>> valpos;
	for(size_t i = 0; i < current_values.size(); i++){
		valpos.push_back(make_pair(i, current_values[i]));
	}
	vector<int> new_order;
	while(valpos.size() > 0){

		pair<int,int> max = valpos[0];
		int pos = 0;
		for(size_t i = 0; i < valpos.size(); i++){
			if(valpos[i].second > max.second){
				max = valpos[i];
				pos = i;
			}			
		}

		valpos.erase(valpos.begin() + pos);
		new_order.push_back(max.first);
	}

	timer.stop();
	return new_order;
}
	
	
std::vector<int> OrderSelecter::compute_order_org_heuristic_rev(State state, AdditiveCartesianHeuristic* heuristic){	
	timer.resume();
	vector<int> ordered_values = compute_order_org_heuristic(state, heuristic);
	vector<int> rev_order;
	for(int i = ordered_values.size() - 1; i >= 0; i--){
		rev_order.push_back(ordered_values[i]);	
	}
	vector<int> new_order;
    vector<int> sat_abs;
	for(int pos : rev_order){
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

void OrderSelecter::print_statistics(){
	cout << "--------- ORDER SELECTER ---------" << endl;
	//cout << "Reorder time: " << timer << endl;
}

}
