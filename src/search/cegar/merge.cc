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
	
	
int Merge::select_smallest_abstractions_and_refine(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int* p2){
	
	cout << "ToRefine: ";
	for(bool b : toRefine){
		cout << b << " ";	
	}
	cout << endl;

	int infinity = EvaluationResult::INFTY;
 	int s1 = infinity;

	cout << "Size: ";
	for(size_t i = 0; i < (*heuristic_functions).size(); i ++){
		cout << (*heuristic_functions)[i]->size() << " ";
		if((*heuristic_functions)[i]->size() < s1 && toRefine[i]){
			s1 = (*heuristic_functions)[i]->size();
			*f1 = (*heuristic_functions)[i];
			*p1 = i;
		}
	}
	cout << endl;
	if(s1 == infinity){
		return -1;	
	}
	
	int s2 = infinity;
	
	for(size_t i = 0; i < (*heuristic_functions).size(); i ++){
		if((*heuristic_functions)[i]->size() < s2 && *p1 != (int)i && toRefine[i]){
			s2 = (*heuristic_functions)[i]->size();
			*f2 = (*heuristic_functions)[i];
			*p2 = i;
		}
	}
	
	if(s2 == infinity){
		return -1;	
	}
	
	cout << "Merge: " << *p1 << " id=" << (*f1)->id << " (" << s1 << ") with : " << *p2 << " id=" << (*f2)->id << " (" << s2 << ")" << endl; 

	return s1 * s2;
}
	
int Merge::select_common_saturation(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2){
	/*
	cout << "ToRefine: ";
	for(bool b : toRefine){
		cout << b << " ";	
	}
	cout << endl;
	*/
	//1. find smallest abstraction which needs to be refined
	int infinity = EvaluationResult::INFTY;
 	int s1 = infinity;

	//cout << "Size: ";
	for(size_t i = 0; i < (*heuristic_functions).size(); i ++){
		//cout << (*heuristic_functions)[i]->size() << " ";
		if((*heuristic_functions)[i]->size() < s1 && toRefine[i]){
			s1 = (*heuristic_functions)[i]->size();
			*f1 = (*heuristic_functions)[i];
			*p1 = i;
		}
	}
	//cout << endl;
	if(s1 == infinity){
		return -1;	
	}
	
	//2. find all abstraction which have an intersection in the saturation
	vector<int> sat_compare = cost_saturation->get_original_cost_partitioning((*f1)->get_abstraction());
	vector<bool> canditates; //(false, (*heuristic_functions).size());
	for(size_t i = 0; i < (*heuristic_functions).size(); i ++){
		if(toRefine[i] && (int)i != *p1){
			vector<int> sat = cost_saturation->get_original_cost_partitioning((*heuristic_functions)[i]->get_abstraction());
			/*
			for(int b : sat_compare){
				cout << b << " ";	
			}
			cout << endl;
			for(int b : sat){
				cout << b << " ";	
			}
			cout << endl;
			*/
			//TODO interset or not interset ?
			if(!intersect(&sat_compare, &sat)){
				//cout << "Candidate: " << i << endl;
				canditates.push_back(true);
			}
			else{
				canditates.push_back(false);
			}
		}
		//cout << "--------------------------------------------------------------------------------" << endl;
	}
	/*		
	cout << "Canditates: ";
	for(bool b : canditates){
		cout << b << " ";	
	}
	cout << endl;	
	*/		   
	//find the smallest candidate
	int s2 = infinity;	
	for(size_t i = 0; i < (*heuristic_functions).size(); i ++){ 
		//cout << "pos: " << i << " c: " << canditates[i] << endl;
		if((*heuristic_functions)[i]->size() < s2 && *p1 != (int)i && canditates[i]){
			//cout << "---> select" << endl;
			s2 = (*heuristic_functions)[i]->size();
			*f2 = (*heuristic_functions)[i];
			*p2 = i;
		}
	}
	
	if(s2 == infinity){
		return -1;	
	}
	
	//cout << "Merge: " << *p1 << " id=" << (*f1)->id << " (" << s1 << ") with : " << *p2 << " id=" << (*f2)->id << " (" << s2 << ")" << endl; 

	return s1 * s2;
	
}

bool Merge::intersect(std::vector<int> *cp1, std::vector<int> *cp2){
	for(size_t i = 0; i < cp1->size(); i++){
		if((*cp1)[i] != 0 && (*cp2)[i] != 0){
			//cout << "Pos: " << i << endl;
			return true;	
		}
	}
	return false;
}
	
int Merge::select_compatible_plan(std::vector<bool> toRefine, CartesianHeuristicFunction** f1, int* p1, CartesianHeuristicFunction** f2, int*p2){
	/*
	cout << "Number of functions: " << (*heuristic_functions).size() << endl;	
	cout << "ToRefine: ";
	for(bool b : toRefine){
		cout << b << " ";	
	}
	cout << endl;
	*/
	//1. find smallest abstraction which needs to be refined
	int infinity = EvaluationResult::INFTY;
 	int s1 = infinity;

	//cout << "Size: ";
	for(size_t i = 0; i < (*heuristic_functions).size(); i ++){
		//cout << (*heuristic_functions)[i]->size() << " ";
		if((*heuristic_functions)[i]->size() < s1 && toRefine[i]){
			s1 = (*heuristic_functions)[i]->size();
			*f1 = (*heuristic_functions)[i];
			*p1 = i;
		}
	}
	//cout << endl;
	if(s1 == infinity){
		return -1;	
	}
	
	//2. find all abstraction which have no compatible plan
	vector<int> sat_compare = cost_saturation->get_original_cost_partitioning((*f1)->get_abstraction());
	vector<bool> canditates; //(false, (*heuristic_functions).size());
	for(size_t i = 0; i < (*heuristic_functions).size(); i ++){
		//cout << "Candidate: " << i << endl;
		if(toRefine[i] && (int)i != *p1){
			CartesianHeuristicFunction* f = (*heuristic_functions)[i];
			//cout << "................ Check compatible " << (*f1)->id << " with " << f->id << "..........." << endl; 
			if(!( (*f1)->get_abstraction()->are_plans_compatible(f->get_abstraction()) || 
				 f->get_abstraction()->are_plans_compatible((*f1)->get_abstraction()))){	
				//cout << "Merge candidate" << endl;
				canditates.push_back(true);
			}
			else{
				canditates.push_back(false);
				//cout << "No Merge candidate" << endl;
			}
		}
		else{
			canditates.push_back(false);
		}
		//cout << "--------------------------------------------------------------------------------" << endl;
	}
		
	cout << "Canditates: ";
	for(bool b : canditates){
		cout << b << " ";	
	}
	cout << endl;	
			   
	//find the smallest candidate
	int s2 = infinity;	
	for(size_t i = 0; i < (*heuristic_functions).size(); i ++){ 
		//cout << "pos: " << i << " c: " << canditates[i] << endl;
		if((*heuristic_functions)[i]->size() < s2 && *p1 != (int)i && canditates[i]){
			//cout << "---> select pos: " << i << endl;
			s2 = (*heuristic_functions)[i]->size();
			*f2 = (*heuristic_functions)[i];
			*p2 = i;
		}
	}
	
	if(s2 == infinity){
		return -1;	
	}
	
	//cout << "Merge: " << *p1 << " id=" << (*f1)->id << " (" << s1 << ") with : " << *p2 << " id=" << (*f2)->id << " (" << s2 << ")" << endl; 

	return s1 * s2;
	
}

bool Merge::merge(vector<bool> toRefine){
	//return false;
	
	merge_timer.resume();
	for (CartesianHeuristicFunction *function : (*heuristic_functions)) {
	     function->set_refined(false);
	}
	CartesianHeuristicFunction* f1 = NULL;
	int p1 = 0;
	CartesianHeuristicFunction* f2 = NULL;
	int p2 = 0;
	
	//Which abstractions should be merged
    //int max_size = select_smallest_abstractions(&f1, &p1, &f2, &p2);
	int max_size = select_compatible_plan(toRefine, &f1, &p1, &f2, &p2);
		
	if(max_size == -1 || max_size > max_merge_size){
		cout << "MAX SIZE MERGE: " << max_size << " < " << max_merge_size << endl;
		merge_timer.stop();
		return false;
	}
    //cout << "Merge: " << p1 << " id=" << f1->id << " with : " << p2 << " id=" << f2->id << endl;
	//Merge abstractions
	f1->merge(f2);
	(*heuristic_functions).erase((*heuristic_functions).begin() + p2); //TODO can not be deleted in cost_saturation
	cost_saturation->remove_abstraction(p2);


	//UPDATE COST PARTITIONING
	int number_orders = cost_saturation->number_of_orders();
	for(int o = 0; o < number_orders; o++){
		//cout << "+++++++++ Order: " << o << " ++++++++++" << endl;
		cost_saturation->recompute_cost_partitioning_unused(o); 
		//Also updates the h values
	}

	//cout << "Functions after merge: ";
	for (CartesianHeuristicFunction *function : (*heuristic_functions)) {
		//cout << function->id << " ";
		cost_saturation->update_h_complete_cost(function->get_abstraction());
		 function->set_refined(false);
	}  
	//cout << endl;
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
