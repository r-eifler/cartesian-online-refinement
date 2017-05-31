#include "cartesian_heuristic_function.h"
#include "utils.h"

using namespace std;

namespace cegar {
   
CartesianHeuristicFunction::CartesianHeuristicFunction(
    const shared_ptr<AbstractTask> &task, Abstraction *abs)
    //Abstraction *abs)
    : //task(task),
      //task_proxy(*task),
      abstraction(abs){
		  
		  max_heuristic = create_max_heuristic(task);
		  //max_heuristic->compute_heuristic_for_cegar(abstraction->get_Task().get_initial_state());
}

int CartesianHeuristicFunction::get_value(const State &parent_state) const {
    //State local_state = task_proxy.convert_ancestor_state(parent_state);
    State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
    //return refinement_hierarchy.get_node(local_state)->get_h_value();
    return abstraction->get_node(local_state)->get_h_value();
}
    
int CartesianHeuristicFunction::online_Refine(const State &parent_state, int max_iter, int max_states_refine) const {
    State local_state = (abstraction->get_Task())->convert_ancestor_state(parent_state);
    return abstraction->onlineRefine(local_state, max_iter, max_states_refine);
}
	
	int CartesianHeuristicFunction::hmax_value(const GlobalState &global_state) const{
	return max_heuristic->compute_heuristic_cegar(global_state);
}
	
	
}
