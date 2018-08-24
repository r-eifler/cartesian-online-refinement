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


int CartesianHeuristicFunction::get_original_h_value(const State &parent_state) const {
	if(abstraction->get_num_states() == 1){
		return 0;	
	}
	State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
    return abstraction->get_node(local_state)->get_original_h_value();
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
    
int CartesianHeuristicFunction::online_Refine(const State &parent_state, std::vector<State> goal_states, int max_iter, int max_states_refine, std::vector<std::vector<int>> *unused_cost) const {
    State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
	vector<State> local_goal_states;
	for(State gs : goal_states){
		local_goal_states.push_back((abstraction->get_Task())->convert_ancestor_state(gs));
	}
    bool refined = abstraction->onlineRefine(local_state, local_goal_states, max_iter, max_states_refine, unused_cost);
	//abstraction->print_states();
	return refined;
}


int CartesianHeuristicFunction::refineBellmanStyle(const State & parent_state){

    State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
	return abstraction->refineBellmanStyle(local_state);
}


int CartesianHeuristicFunction::refineSplitPre(const State & state, const State & prestate){

    State local_state = (abstraction->get_Task())->convert_ancestor_state(state);
    State local_prestate = (abstraction->get_Task())->convert_ancestor_state(prestate);
	return abstraction->refineSplitPre(local_state, local_prestate);
}


int CartesianHeuristicFunction::refineSplitPreAction(const State &state, const State &preState, const std::vector<std::pair<int,int>> conditions){

    State local_state = (abstraction->get_Task())->convert_ancestor_state(state);
    State local_prestate = (abstraction->get_Task())->convert_ancestor_state(preState);
	return abstraction->refineSplitOnCondition(local_state, local_prestate, conditions);
}

int CartesianHeuristicFunction::refineNewGoal(const State &state, const State &new_goal){

    State local_state = (abstraction->get_Task())->convert_ancestor_state(state);
    State local_goal = (abstraction->get_Task())->convert_ancestor_state(new_goal);
	return abstraction->refineNewGoal(local_state, local_goal);
}

int CartesianHeuristicFunction::refineBasedOnBellman(const State &state, const int bound){

    State local_state = (abstraction->get_Task())->convert_ancestor_state(state);
	return abstraction->refineBasedOnBellman(local_state, bound);
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
