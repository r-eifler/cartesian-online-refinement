#include "merge.h"

#include <sstream>
#include <iostream>
#include <unordered_set>

using namespace std;

namespace cegar {
Merge::Merge(CostSaturation* cs, utils::RandomNumberGenerator* r) :
	cost_saturation(cs), 
	rng(r){
	merge_timer.stop();
}

void Merge::set_heuristic_functions(std::vector<CartesianHeuristicFunction*> *fv){
	heuristic_functions = fv;
}

int Merge::select_smallest_abstractions(CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int* p2){

 	int s1 = (*heuristic_functions)[0]->size();
        *f1 = (*heuristic_functions)[0];
        *p1 = 0;
        
        for(size_t i = 0; i < (*heuristic_functions).size(); i ++){
            if((*heuristic_functions)[i]->size() < s1){
                s1 = (*heuristic_functions)[i]->size();
                *f1 = (*heuristic_functions)[i];
                *p1 = i;
            }
        }
        int s2 = (*heuristic_functions)[0]->size();
        *f2 = (*heuristic_functions)[0];
        *p2 = 0;
        if(*p1 == 0){
            s2 = (*heuristic_functions)[1]->size();
            *f2 = (*heuristic_functions)[1];
            *p2 = 1;
        }
        
        for(size_t i = 0; i < (*heuristic_functions).size(); i ++){
            if((*heuristic_functions)[i]->size() < s2 && *p1 != (int)i){
                s2 = (*heuristic_functions)[i]->size();
                *f2 = (*heuristic_functions)[i];
                *p2 = i;
            }
        }
	cout << "Merge: " << *p1 << " id=" << (*f1)->id << " (" << s1 << ") with : " << *p2 << " id=" << (*f2)->id << " (" << s2 << ")" << endl; 

	return s1 * s2;
}

bool Merge::merge(){
	merge_timer.resume();
	for (CartesianHeuristicFunction *function : (*heuristic_functions)) {
	     function->set_refined(false);
	}
	CartesianHeuristicFunction* f1 = NULL;
	int p1 = 0;
	CartesianHeuristicFunction* f2 = NULL;
	int p2 = 0;
	
	//Which abstractions should be merged
        int max_size = select_smallest_abstractions(&f1, &p1, &f2, &p2);
	cout << "MAX SIZE MERGE: " << max_size << " < " << max_merge_size << endl;
	if(max_size > max_merge_size){
		merge_timer.stop();
		return false;
	}
        
        //Merge abstractions
        f1->merge(f2);
        (*heuristic_functions).erase((*heuristic_functions).begin() + p2);
        cost_saturation->remove_abstraction(p2);
        
        
        //UPDATE COST PARTITIONING
        int number_orders = cost_saturation->number_of_orders();
	    for(int o = 0; o < number_orders; o++){
            //cout << "+++++++++ Order: " << o << " ++++++++++" << endl;
            cost_saturation->recompute_cost_partitioning_unused(o); 
            //Also updates the h values
        }

        for (CartesianHeuristicFunction *function : (*heuristic_functions)) {
             function->set_refined(false);
        }  
	merge_timer.stop();    
	merged_abs++;
	return true;	
}

void Merge::print_statistics(){
	cout << "----------- MERGE --------------" << endl;
	cout << "Merged abstractions: " << merged_abs << endl;
	cout << "Merge time: " << merge_timer << endl;
}

}
