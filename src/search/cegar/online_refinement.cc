#include "online_refinement.h"

#include <sstream>
#include <iostream>
#include <unordered_set>

using namespace std;

namespace cegar {
OnlineRefinement::OnlineRefinement(CostSaturation* cs, utils::RandomNumberGenerator* r, int mso) :
	cost_saturation(cs), 
	rng(r),
	max_states_online(mso){


	timer.stop();
}

void OnlineRefinement::set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv){
	heuristic_functions = fv;
}

bool OnlineRefinement::refine(State state, std::vector<bool> toRefine){
	timer.resume();
	//cout << "ONLINE Refinement refine" << endl;
    if(max_states_online - online_refined_states <= 0){ 
		//maximal number of online refined states reached ?
		//cout << "Max states reached" << endl;
		return false;
	}
        
	bool refined = false;
        
        for(size_t i = 0; i < toRefine.size(); i++){
            if(false)
                cout << "------------Refine " << i << " ----------------------" << endl;
            if(toRefine[i]){              
               int refined_states = (*heuristic_functions)[i]->online_Refine(state, 5, false, max_states_online - online_refined_states);
               if(refined_states > 0){
                    refined = true;  
                    if(false)
                        cout << "--> Refined" << endl;
               }
               online_refined_states += refined_states;           
            }
        }

	if(!refined){
		timer.stop();
		return false;
	}
	
	if(false)
		cout << "recompute cost partitioning" << endl;


	//Recompute costpartitioning, Reset Refined status, update h values
	int number_orders = cost_saturation->number_of_orders();
	for(int o = 0; o < number_orders; o++){
            //cout << "+++++++++ Order: " << o << " ++++++++++" << endl;
            cost_saturation->recompute_cost_partitioning_unused(o); //has to be executed in this order otherwise the wrong task is set in the abstraktion
	/*
            for (CartesianHeuristicFunction *function : (*heuristic_functions)) {
                function->update_h_and_g_values(o, false);
            }
		*/
        }
        for (CartesianHeuristicFunction *function : (*heuristic_functions)) {
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
