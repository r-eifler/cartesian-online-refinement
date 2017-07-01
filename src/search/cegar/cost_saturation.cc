#include "cost_saturation.h"


#include "cartesian_heuristic_function.h"
#include "subtask_generators.h"
#include "utils.h"

#include "../globals.h"
#include "../task_tools.h"

#include "../causal_graph.h"

#include "../tasks/modified_operator_costs_task.h"

#include "../utils/countdown_timer.h"
#include "../utils/logging.h"
#include "../utils/memory.h"

#include <algorithm>
#include <cassert>

using namespace std;

namespace cegar {
/*
  We reserve some memory to be able to recover from out-of-memory
  situations gracefully. When the memory runs out, we stop refining and
  start the next refinement or the search. Due to memory fragmentation
  the memory used for building the abstraction (states, transitions,
  etc.) often can't be reused for things that require big continuous
  blocks of memory. It is for this reason that we require such a large
  amount of memory padding.
*/
static const int memory_padding_in_mb = 75;

CostSaturation::CostSaturation(
    vector<shared_ptr<SubtaskGenerator>> &subtask_generators,
    int max_states,
    int max_non_looping_transitions,
    double max_time,
    bool use_general_costs,
    PickSplit pick_split,
    utils::RandomNumberGenerator &rng)
    : subtask_generators(subtask_generators),
      max_states(max_states),
      max_non_looping_transitions(max_non_looping_transitions),
      max_time(max_time),
      use_general_costs(use_general_costs),
      pick_split(pick_split),
      rng(rng),
      num_abstractions(0),
      num_states(0),
      num_non_looping_transitions(0) {
}

//Builds heuristics functions
vector<CartesianHeuristicFunction*> CostSaturation::generate_heuristic_functions(
    const shared_ptr<AbstractTask> &task) {
    // For simplicity this is a member object. Make sure it is in a valid state.
    assert(heuristic_functions.empty());
	
		
	
	abstask = task;

    utils::CountdownTimer timer(max_time);

    TaskProxy task_proxy(*task);
	
	//Build causal graph
	const CausalGraph c_graph =  get_causal_graph(&(*task));
	cout << "+++++++++++++++++++ Causal Graph ++++++++++++++++++" << endl;
	c_graph.dump(task_proxy);
	cout << "+++++++++++++++++++ Causal Graph ++++++++++++++++++" << endl;

	cout << "Operators" << endl;
      for (OperatorProxy op : task_proxy.get_operators())
          cout << op.get_name() << endl;
	cout << "Variables: " << endl;
	VariablesProxy vars = task_proxy.get_variables();
	for(uint i = 0; i < vars.size(); i++){
		cout << vars[i].get_name() <<  " domain size: " << vars[i].get_domain_size() << endl;	
	}
	
	cout << "+++++++++++++++++++ Goals ++++++++++++++++++" << endl;
     GoalsProxy goals = task_proxy.get_goals();
     for(uint i = 0; i < goals.size(); i++){
		cout << goals[i].get_variable().get_name() <<  " = " << goals[i].get_value() << endl;	
	}
	

    verify_no_axioms(task_proxy);
    verify_no_conditional_effects(task_proxy);

    reset(task_proxy);

    State initial_state = TaskProxy(*task).get_initial_state();

    function<bool()> should_abort =
        [&] () {
            return num_states >= max_states ||
                   num_non_looping_transitions >= max_non_looping_transitions ||
                   timer.is_expired() ||
                   !utils::extra_memory_padding_is_reserved() ||
                   state_is_dead_end(initial_state);
        };

    utils::reserve_extra_memory_padding(memory_padding_in_mb);
    for (shared_ptr<SubtaskGenerator> subtask_generator : subtask_generators) {
        SharedTasks subtasks = subtask_generator->get_subtasks(task);
        build_abstractions(subtasks, timer, should_abort);
        if (should_abort())
            break;
    }
    
    //TODO memory padding
    //if (utils::extra_memory_padding_is_reserved())
    //    utils::release_extra_memory_padding();
    print_statistics();

    vector<CartesianHeuristicFunction*> functions;
	for(size_t i = 0; i < heuristic_functions.size(); i++){
		functions.push_back(&heuristic_functions[i]);
	}
    //swap(heuristic_functions, functions);
    return functions;
}

void CostSaturation::reset(const TaskProxy &task_proxy) {
    remaining_costs = get_operator_costs(task_proxy);
    num_abstractions = 0;
    num_states = 0;
}
	
void CostSaturation::add_cost_partitioning(std::vector<int>* cost1, std::vector<int>* cost2){
	
	for(size_t i = 0; i < cost1->size(); i++){
		(*cost1)[i] = (*cost1)[i] + (*cost2)[i];	
	}
}

void CostSaturation::reduce_remaining_costs(
    const vector<int> &saturated_costs) {
    assert(remaining_costs.size() == saturated_costs.size());
    for (size_t i = 0; i < remaining_costs.size(); ++i) {
        int &remaining = remaining_costs[i];
        const int &saturated = saturated_costs[i];
        assert(saturated <= remaining);
        /* Since we ignore transitions from states s with h(s)=INF, all
           saturated costs (h(s)-h(s')) are finite or -INF. */
        assert(saturated != INF);
        if (remaining == INF) {
            // INF - x = INF for finite values x.
        } else if (saturated == -INF) {
            remaining = INF;
        } else {
            remaining -= saturated;
        }
        assert(remaining >= 0);
    }
}

shared_ptr<AbstractTask> CostSaturation::get_remaining_costs_task(
    shared_ptr<AbstractTask> &parent) const {
    vector<int> costs = remaining_costs;
    return make_shared<extra_tasks::ModifiedOperatorCostsTask>(
        parent, move(costs));
}
	
shared_ptr<AbstractTask> CostSaturation::get_remaining_costs_task(shared_ptr<AbstractTask> &parent,  vector<int> old_costs){
    
	
	for(uint i = 0; i < remaining_costs.size(); i++){
		remaining_costs[i] = remaining_costs[i] + old_costs[i];	
	}
	
	vector<int> costs = remaining_costs;
	
    return make_shared<extra_tasks::ModifiedOperatorCostsTask>(
        parent, move(costs));
}

bool CostSaturation::state_is_dead_end(const State &state) const {
    for (const CartesianHeuristicFunction &function : heuristic_functions) {
        if (function.get_value(state) == INF)
            return true;
    }
    return false;
}

void CostSaturation::build_abstractions(
    const vector<shared_ptr<AbstractTask>> &subtasks,
    const utils::CountdownTimer &timer,
    function<bool()> should_abort) {
    int rem_subtasks = subtasks.size();
    for (shared_ptr<AbstractTask> subtask : subtasks) {
		
        subtask = get_remaining_costs_task(subtask);
        assert(num_states < max_states);
        Abstraction *abstraction = new Abstraction(
            subtask,
            max(1, (max_states - num_states) / rem_subtasks),
            max(1, (max_non_looping_transitions - num_non_looping_transitions) /
                rem_subtasks),
            timer.get_remaining_time() / rem_subtasks,
            use_general_costs,
            pick_split,
            rng);
		
		

        ++num_abstractions;
        num_states += abstraction->get_num_states();
        num_non_looping_transitions += abstraction->get_num_non_looping_transitions();
        assert(num_states <= max_states);
		
		
		cout << "Remaining Cost: " << endl;
		for(int i : remaining_costs){
			cout << i << " ";	
		}
		cout << endl;
		
		cout << "Cost Partitioning" << endl;
		vector<int> sturated_cost = abstraction->get_saturated_costs();
		for(int i : sturated_cost){
			cout << i << " ";	
		}
		cout << endl;
		
		
        reduce_remaining_costs(sturated_cost);
        //int init_h = abstraction->get_h_value_of_initial_state();
        //if (init_h > 0) {
			abstractions.push_back(abstraction);
          	heuristic_functions.emplace_back(abstraction, abstractions.size()-1);
		  
        //}
        if (should_abort())
            break;

        --rem_subtasks;
    }
}
	
void CostSaturation::recompute_cost_partitioning(){
	//cout << "recompute cost partitioning" << endl;
	
	//TaskProxy task_proxy(*abstask); 
	
	//reset ramaining cost;
	//remaining_costs = get_operator_costs(task_proxy);
	//cout << "remainig cost reset" << endl;

	for(Abstraction* abs : abstractions){
		//cout << "-------------Refined: " << abs->refined  << " ---------------------" << endl;
		if(!abs->refined){
			//cout << "not refined" << endl;
			continue;	
		}
		abs->refined = false;
		
		vector<int> sturated_cost_old = abs->get_costs_partitioning();
		
		/*
		cout << "Remaining Cost: " << endl;
		for(int i : remaining_costs){
			cout << i << " ";	
		}
		cout << endl;
		cout << "Old Saturated cost" << endl;
		for(int i : sturated_cost_old){
			cout << i << " ";	
		}
		cout << endl;
		*/
		
		
		//update the task in the abstraction
		shared_ptr<AbstractTask> subtask = abs->get_AbsTask();
		subtask = get_remaining_costs_task(subtask, sturated_cost_old);
		abs->update_Task(subtask);
		
		//Update the remaining cost of the states aber refinement
		abs->update_h_values();
		
		vector<int> sturated_cost = abs->get_saturated_costs();
		/*
		cout << "Saturated cost" << endl;		
		for(int i : sturated_cost){
			cout << i << " ";	
		}
		cout << endl;*/
		reduce_remaining_costs(sturated_cost);
		/*
		cout << "Remaining Cost: " << endl;
		for(int i : remaining_costs){
			cout << i << " ";	
		}
		cout << endl;
		*/
		
		
		//vector<int> sturated_cost = abs->get_saturated_costs();
		//reduce_remaining_costs(sturated_cost);
		//cout << "******************************************************************" << endl;
	}
}
	
void CostSaturation::rise_heuristic(int pos){	
	assert(pos > 0);
	Abstraction* abs_up = abstractions[pos];
	Abstraction* abs_down = abstractions[pos-1];
	
	//abs_up can now also use the cost in abs_down
	vector<int> cost_up = abs_up->get_costs_partitioning();
	/*cout << "Cost UP " << cost_up.size() << endl;
	for(size_t n = 0; n < cost_up.size(); n++){
			cout << cost_up[n] << " ";	
	}
	cout << endl;*/
	vector<int> cost_down = abs_down->get_costs_partitioning();	
	/*cout << "Cost DOWN" << endl;
	for(size_t n = 0; n < cost_down.size(); n++){
			cout << cost_down[n] << " ";	
	}
	cout << endl;*/
	add_cost_partitioning(&cost_up, &cost_down);
	/*cout << "SUM" << cost_up.size() << endl;
	for(size_t n = 0; n < cost_up.size(); n++){
			cout << cost_up[n] << " ";	
	}
	cout << endl;*/
	
	//update the task in the abstraction and recopute the distances
	shared_ptr<AbstractTask> subtask = abs_up->get_AbsTask();
	subtask = get_remaining_costs_task(subtask, cost_up);
	abs_up->update_Task(subtask);
	abs_up->update_h_values();

	
	//update remaining cost
	cost_up = abs_up->get_saturated_costs();
	/*cout << "Cost UP after" << endl;
	for(size_t n = 0; n < cost_up.size(); n++){
			cout << cost_up[n] << " ";	
	}
	cout << endl;*/
	reduce_remaining_costs(cost_up);
	
	//recompute costpartitioning of abs down
	subtask = abs_down->get_AbsTask();
	subtask = get_remaining_costs_task(subtask);
	abs_down->update_Task(subtask);
	abs_down->update_h_values();	
	
	cost_down = abs_down->get_saturated_costs();
	reduce_remaining_costs(cost_down);
	/*cout << "Cost DOWN after" << endl;
	for(size_t n = 0; n < cost_down.size(); n++){
			cout << cost_down[n] << " ";	
	}
	cout << endl;*/
	
	//change pointer
	iter_swap(abstractions.begin() + pos, abstractions.begin() + pos - 1);
}
	
void CostSaturation::remove_abstraction(int pos){
	
	Abstraction* abs = abstractions[pos];
	vector<int> cost = abs->get_costs_partitioning();
	add_cost_partitioning(&remaining_costs, &cost);
	abstractions.erase(abstractions.begin() + pos);
}

void CostSaturation::print_statistics() const {
    g_log << "Done initializing additive Cartesian heuristic" << endl;
    cout << "Cartesian abstractions built: " << num_abstractions << endl;
    cout << "Cartesian heuristic functions stored: "
         << heuristic_functions.size() << endl;
    cout << "Cartesian states: " << num_states << endl;
    cout << "Total number of non-looping transitions: "
         << num_non_looping_transitions << endl;
    cout << endl;
	
	float unused_cost = 0;
	cout << "Remaining Cost: " << endl;
		for(int i : remaining_costs){
			if(i > 0){
				unused_cost++;	
			}
			cout << i << " ";	
		}
	cout << endl;
	cout << "Unused Cost: " << (unused_cost / remaining_costs.size()) << endl;
}
}
