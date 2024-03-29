#include "cartesian_heuristic_function.h"
#include "utils.h"

using namespace std;

namespace cegar {
   
CartesianHeuristicFunction::CartesianHeuristicFunction(Abstraction *abs, int i)
    :abstraction(abs), id(i){		  
}

int CartesianHeuristicFunction::get_value(const State &parent_state) const {
	if(abstraction->get_num_states() == 1){
		return 0;	
	}
    //State local_state = task_proxy.convert_ancestor_state(parent_state);
    State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
    //return refinement_hierarchy.get_node(local_state)->get_h_value();
    return abstraction->get_node(local_state)->get_h_value();
}
	
int CartesianHeuristicFunction::get_original_value(const State &parent_state) const {
	if(abstraction->get_num_states() == 1){
		return 0;	
	}
	State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
    return abstraction->get_node(local_state)->get_c_h();
}
	
int CartesianHeuristicFunction::get_value(const State &parent_state, int order) const{
	if(abstraction->get_num_states() == 1){
		return 0;	
	}
	State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
	return abstraction->get_node(local_state)->get_h_value(order);
}
	
std::vector<int> CartesianHeuristicFunction::get_values(const State &parent_state) const {
	if(abstraction->get_num_states() == 1){
		vector<int> res;
		for(int i = 0; i < abstraction->get_num_orders(); i++){
			res.push_back(0);	
		}
		return res;
	}
	 State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
	return abstraction->get_node(local_state)->get_h_values();
}
    
int CartesianHeuristicFunction::online_Refine(const State &parent_state, int max_iter, int update_h_values, int max_states_refine, std::vector<std::vector<int>> *unused_cost) const {
    State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
    return abstraction->onlineRefine(local_state, max_iter, update_h_values, max_states_refine, unused_cost);
}

bool CartesianHeuristicFunction::satisfies_goal(State parent_state){
	State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
	return abstraction->satisfies_goal(local_state);	
}
		
void CartesianHeuristicFunction::print_statistics() const{
	abstraction->print_end_statistics();	
}
	
void CartesianHeuristicFunction::update_h_values() {
	abstraction->update_h_values();
}
	
void CartesianHeuristicFunction::update_h_and_g_values(int pos, bool new_order){
	abstraction->update_h_and_g_values(pos, new_order);	
}

void CartesianHeuristicFunction::merge(CartesianHeuristicFunction *function) {
	abstraction->merge(function->abstraction);
}
	
void CartesianHeuristicFunction::set_refined(bool b){
	abstraction->refined = b;
}
	
int CartesianHeuristicFunction::size(){
	return abstraction->get_num_states();	
}	
	
Abstraction* CartesianHeuristicFunction::get_abstraction(){
	return abstraction;	
}
	
}
