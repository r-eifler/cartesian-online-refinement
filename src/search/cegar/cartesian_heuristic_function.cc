#include "cartesian_heuristic_function.h"
#include "utils.h"

using namespace std;

namespace cegar {
   
CartesianHeuristicFunction::CartesianHeuristicFunction(Abstraction *abs)
    :abstraction(abs){		  
}

int CartesianHeuristicFunction::get_value(const State &parent_state) const {
    //State local_state = task_proxy.convert_ancestor_state(parent_state);
    State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
    //return refinement_hierarchy.get_node(local_state)->get_h_value();
    return abstraction->get_node(local_state)->get_h_value();
}
    
int CartesianHeuristicFunction::online_Refine(const State &parent_state, int max_iter, int update_h_values, int max_states_refine) const {
    State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
    return abstraction->onlineRefine(local_state, max_iter, update_h_values, max_states_refine);
}
	
		
	void CartesianHeuristicFunction::print_statistics() const{
		abstraction->print_end_statistics();	
	}
	
	
}
