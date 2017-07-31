#include "online_refinement.h"

#include <sstream>
#include <iostream>
#include <unordered_set>

using namespace std;

namespace cegar {
OnlineRefinement::OnlineRefinement(CostSaturation* cs, utils::RandomNumberGenerator* r, int mso, bool use_us) :
	cost_saturation(cs), 
	rng(r),
	max_states_online(mso),
	use_usefull_split(use_us){

	timer.stop();
}

void OnlineRefinement::set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv){
	heuristic_functions = fv;
}

bool OnlineRefinement::refine(State state, std::vector<bool> toRefine){
	timer.resume();
    if(max_states_online - online_refined_states <= 0){ 
		//maximal number of online refined states reached ?
		//cout << "Max states reached" << endl;
		return false;
	}
        
	bool refined = false;
    std::vector<std::vector<int>> *unused_cost = NULL;
	if(use_usefull_split){
		unused_cost = cost_saturation->get_unused_cost();
	}
	for(size_t i = 0; i < toRefine.size(); i++){
		//cout << "Refine: " << (*heuristic_functions)[i]->id << " toRefine " << toRefine[i] << endl;
		if(toRefine[i]){  		
			
		   int refined_states = (*heuristic_functions)[i]->online_Refine(state, 5, false, max_states_online - online_refined_states, unused_cost);
			//cout << "Refined states: " << refined_states << endl;
		   if(refined_states > 0){				    
				refined = true;  
		   }
		   online_refined_states += refined_states;           
		}
	}

	if(!refined){
		timer.stop();
		return false;
	}

	//Recompute costpartitioning, Reset Refined status, update h values
	int number_orders = cost_saturation->number_of_orders();
	for(int o = 0; o < number_orders; o++){
    	cost_saturation->recompute_cost_partitioning_unused(o); 
	}
	for (CartesianHeuristicFunction *function : (*heuristic_functions)) {
		cost_saturation->update_h_complete_cost(function->get_abstraction());
		 function->set_refined(false);
	}
	timer.stop();
	return true;
}

void OnlineRefinement::print_statistics(){
	cout << "--------- ONLINE REFINEMENT ---------" << endl;
	cout << "Online refined states: " << online_refined_states << "/" << max_states_online << endl;
	cout << "Refine time: " << timer << endl;
}

}
