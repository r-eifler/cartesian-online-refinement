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
#include <cmath>

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
	bool use_all_goals,
    PickSplit pick_split,
    utils::RandomNumberGenerator &rng)
    : subtask_generators(subtask_generators),
      max_states(max_states),
      max_non_looping_transitions(max_non_looping_transitions),
      max_time(max_time),
      use_general_costs(use_general_costs),
	  use_all_goals(use_all_goals),
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
	/*
	const CausalGraph c_graph =  get_causal_graph(&(*task));
	cout << "+++++++++++++++++++ Causal Graph ++++++++++++++++++" << endl;
	c_graph.dump(task_proxy);
	cout << "+++++++++++++++++++ Causal Graph ++++++++++++++++++" << endl;
	
	
	cout << "Operators" << endl;
	int op_n = 0;
      for (OperatorProxy op : task_proxy.get_operators())
          cout << op_n++ << " " << op.get_name() << endl;
	*/
	
	
	/*
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
	*/
	

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
	
	
	
	vector<int> order;
	cout << "Initial ORDER: ";
	for(uint i = 0; i < heuristic_functions.size(); i++){
		cout << i << " ";	
		order.push_back(i);
	}
	cout << endl;
	scp_orders.push_back(order);
	vector<int> rm_copy(remaining_costs);
	remaining_costs_order.push_back(rm_copy);

    vector<CartesianHeuristicFunction*> functions;
	for(size_t i = 0; i < heuristic_functions.size(); i++){
		functions.push_back(&heuristic_functions[i]);
	}
	
	//Add again all goals to every abstraction
	if(use_all_goals){
		cout << "Add again all goals to every abstraction" << endl;
		for(Abstraction* abs : abstractions){
			abs->addGoals(task_proxy.get_goals());	
		}
		recompute_cost_partitioning_unused(0);
	}
	
	//Compute average abstraction size
	int abs_size_total = 0;
	for(Abstraction* abs : abstractions){
		abs_size_total += abs->get_num_states();
	}
	cout << "Average abstratcion size begin: " << (abs_size_total / abstractions.size()) << endl;
	
    //swap(heuristic_functions, functions);
	/*
	for(Abstraction* abs : abstractions){
		abs->print_states();
		vector<int> c = get_original_cost_partitioning(abs);
		for(int o : c){
			cout << o << " ";	
		}
		cout << endl;
		abs->print_cost();
	}
	*/
    return functions;
}
	
int CostSaturation::compute_threshold(){
	TaskProxy task_proxy(*abstask);
	int op_n = 0;
	//long long cost_mul = 1;
	long cost_add = 0;
    for (OperatorProxy op : task_proxy.get_operators()){
    	op_n++;
		//cost_mul *= op.get_cost();
		cost_add += op.get_cost();
	}
	//cout << pow(cost_mul, 1.0/op_n) << endl;
	//cout << cost_add / op_n << endl;
	return cost_add / op_n;
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
	
void CostSaturation::reduce_remaining_costs(
    const vector<int> &saturated_costs, int order) {
    assert(remaining_costs_order[order].size() == saturated_costs.size());
    for (size_t i = 0; i < remaining_costs_order[order].size(); ++i) {
        int &remaining = remaining_costs_order[order][i];
        const int &saturated = saturated_costs[i];
        assert(saturated <= remaining);
        /* Since we ignore transitions from states s with h(s)=INF, all
           saturated costs (h(s)-h(s')) are finite or -INF. */
        assert(saturated != INF);
        if (remaining == INF) {
            // INF - x = INF for finite values x.
        } else if (saturated == -INF) {
            remaining_costs_order[order][i] = INF;
        } else {
            remaining_costs_order[order][i] -= saturated;
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
	
shared_ptr<AbstractTask> CostSaturation::get_remaining_costs_task(shared_ptr<AbstractTask> &parent,  vector<int> old_costs, int order){
    
	
	for(uint i = 0; i < remaining_costs_order[order].size(); i++){
		remaining_costs_order[order][i] = remaining_costs_order[order][i] + old_costs[i];	
	}
	
	vector<int> costs = remaining_costs_order[order];
	
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
		
		/*
		//check if goals fact has already been used
		FactPair goal_fact = get_fact_pairs(task_proxy = TaskProxy(*subtask))[0];
		for(pair<int,int> g : used_goals){
			   if(g.first
		}
		*/
		cout << "-----------------ABSTRACTION " << abstractions.size() << "--------------------------" << endl;
		cout << "MEMORY: " << utils::extra_memory_padding_is_reserved() << endl;
		
        subtask = get_remaining_costs_task(subtask);
        assert(num_states < max_states);
        Abstraction *abstraction = new Abstraction(
            subtask,
            subtask,
            max(1, (max_states - num_states) / rem_subtasks),
            max(1, (max_non_looping_transitions - num_non_looping_transitions) /
                rem_subtasks),
            timer.get_remaining_time() / rem_subtasks,
            use_general_costs,
            pick_split,
            rng);
		
		//if memory limit has been reached delete abstraction
		if (should_abort()){
			delete abstraction;
            break;			
		}

		
	

        ++num_abstractions;
        num_states += abstraction->get_num_states();
        num_non_looping_transitions += abstraction->get_num_non_looping_transitions();
        assert(num_states <= max_states);
		
		vector<int> sturated_cost = abstraction->get_saturated_costs(0);

        reduce_remaining_costs(sturated_cost);
		//TODO which abstractions should be stored ?
        //int init_h = abstraction->get_h_value_of_initial_state();
        //if (init_h > 0) {		
		if(abstraction->get_num_states() > 1){
			abstractions.push_back(abstraction);
          	heuristic_functions.emplace_back(abstraction, abstractions.size()-1);
		  
        }
		else{
			delete abstraction;
		}
        if (should_abort())
            break;

        --rem_subtasks;
    }
}

 void CostSaturation::recompute_cost_partitioning_unused_all(){
	for(size_t o = 0; o < scp_orders.size(); o++){
		recompute_cost_partitioning_unused(o);
	}
 }
	
void CostSaturation::recompute_cost_partitioning_unused(int order_id){
	
	//cout << "recompute_cost_partitioning_unused: ORDER " << order_id  << endl;
	/*
	cout << "EXISTING ORDERS REMAINING COST: " << endl;
	for(size_t i = 0; i < remaining_costs_order.size(); i++){
		cout << "Remaining Cost: " << i << endl;
		for(int j : remaining_costs_order[i]){
			cout << j << " ";	
		}
		cout << endl;
	}
	*/
	/*
	cout << "Order: ";
	for(int p : scp_orders[order_id]){
		cout << p << " ";	
	}
	cout << endl;
	*/
	
	for(int pos : scp_orders[order_id]){
		Abstraction* abs = abstractions[pos];
		//cout << "------------- Recompute unused cost abs: " << pos << " refined " << abs->refined  << " ---------------------" << endl;
		
		//As the refinement of an abstraction can also derease the cost of un action 
		//all abstractions are recomputed
		//if(!abs->refined){
			//cout << "not refined" << endl;
		//	continue;	
		//}
		
		//abs->refined = false; //TODO
		vector<int> sturated_cost_old = abs->get_costs_partitioning(order_id);
		
		/*
		cout << "Old Saturated cost" << endl;
		for(int i : sturated_cost_old){
			cout << i << " ";	
		}
		cout << endl;
		*/
		/*
		cout << "Remaining Cost: " << endl;
		*/
		/*
		for(uint i = 0; i < remaining_costs_order[order_id].size(); i++){
			//cout << remaining_costs_order[order_id][i] << " ";
			//assert(! (remaining_costs_order[order_id][i] == 1 && sturated_cost_old[i] == 1)); //only for unit cost
				
		}
		*/
		//cout << endl;
				
		//update the task in the abstraction
		shared_ptr<AbstractTask> subtask = abs->get_OriginalAbsTask();
		subtask = get_remaining_costs_task(subtask, sturated_cost_old, order_id);
		abs->update_Task(subtask);
		
		//Update the remaining cost of the states aber refinement
		abs->update_h_values();
		
		vector<int> sturated_cost = abs->get_saturated_costs(order_id);

		/*
		cout << "New Saturated cost" << endl;		
		for(int i : sturated_cost){
			cout << i << " ";	
		}
		cout << endl;
		*/
		reduce_remaining_costs(sturated_cost, order_id);
		
		/*
		cout << "New Remaining Cost: " << endl;
		for(int i : remaining_costs_order[order_id]){
			cout << i << " ";	
		}
		cout << endl;
		
		*/
		
		abs->update_h_and_g_values(order_id, false); //TOOD does not need to be performed elsewehere anymore
	}
}
	
	
void CostSaturation::recompute_cost_partitioning(int order_id){
	//cout << "recompute_cost_partitioning Number of orders: " << scp_orders.size() << endl;
	assert((uint)order_id < scp_orders.size());
	recompute_cost_partitioning(scp_orders[order_id]);
}

void CostSaturation::recompute_cost_partitioning(std::vector<int> order){
	/*
	cout << "recompute_cost_partitioning: testwise ";
	for(int p : order){
		cout << p << " ";	
	}
	cout << endl;
	*/
	TaskProxy task_proxy(*abstask); 
	
	//reset ramaining cost;
	remaining_costs = get_operator_costs(task_proxy);
	

	for(int pos : order){
		
		Abstraction* abs = abstractions[pos];
		
		/*
		cout << "Remaining Cost: " << endl;
		for(int i : remaining_costs){
			cout << i << " ";	
		}
		cout << endl;
		*/
		/*
		cout << "--------Pos " << pos << "----------" << endl;
		cout << "Current Saturation" << endl;
			 cout << "Abs " << pos << ":";
			abs->print_current_cost();	
		
		*/
		
		
		//update the task in the abstraction
		shared_ptr<AbstractTask> subtask = abs->get_OriginalAbsTask();
		subtask = get_remaining_costs_task(subtask);
		abs->update_Task(subtask);
		
		//Update the remaining cost of the states aber refinement
		abs->update_h_values();
		
		vector<int> sturated_cost = abs->get_saturated_costs(-1);
		
		//cout << "Saturated cost" << endl;	
		
		
		/*
		for(int i : sturated_cost){
			cout << i << " ";	
		}
		cout << endl;
		*/
		reduce_remaining_costs(sturated_cost);
		/*
		cout << "Remaining Cost: " << endl;
		for(int i : remaining_costs){
			cout << i << " ";	
		}
		cout << endl;
		*/
		/*
		cout << "New Saturation" << endl;
			 cout << "Abs " << pos << ":";
			abs->print_current_cost();	
		
		*/
	}	
	/*
	int abs_n = 0;
	cout << "Current Saturation all" << endl;
	for(Abstraction* abs : abstractions){
		cout << "Abs " << abs_n++ << ":";
		abs->print_current_cost();	
	}
	cout << "******************************************************************" << endl;
	*/
	/*
	cout << "EXISTING ORDERS REMAINING COST TESTWISE: " << endl;
	for(size_t i = 0; i < remaining_costs_order.size(); i++){
		cout << "Remaining Cost: " << i << endl;
		for(int j : remaining_costs_order[i]){
			cout << j << " ";	
		}
		cout << endl;
	}
	*/
	
}
	
std::vector<int> CostSaturation::get_original_cost_partitioning(Abstraction* abs){
	TaskProxy task_proxy(*abstask); 
	
	//reset ramaining cost;
	remaining_costs = get_operator_costs(task_proxy);
	
	//update the task in the abstraction
	shared_ptr<AbstractTask> subtask = abs->get_OriginalAbsTask();
	subtask = get_remaining_costs_task(subtask);
	abs->update_Task(subtask);

	//Update the remaining cost of the states after refinement
	abs->update_h_values();

	return abs->get_saturated_costs(-1);	
}
	
void CostSaturation::update_h_complete_cost(Abstraction* abs){
	
	TaskProxy task_proxy(*abstask); 
	
	//reset ramaining cost;
	remaining_costs = get_operator_costs(task_proxy);
	
	//update the task in the abstraction
	shared_ptr<AbstractTask> subtask = abs->get_OriginalAbsTask();
	subtask = get_remaining_costs_task(subtask);
	abs->update_Task(subtask);

	//Update the remaining cost of the states after refinement
	abs->update_h_values_complete_cost();
}
	
int CostSaturation::number_of_orders(){
	return 	scp_orders.size();
}
	
std::vector<int> CostSaturation::get_order(int id){
	assert((uint) id < scp_orders.size());
	return scp_orders[id];	
}
void CostSaturation::update_order(int id, std::vector<int> new_order){
	assert((uint)id < scp_orders.size());
	scp_orders[id] = new_order;
}
	
//The added order needs to be the one which has been computed testwise before TODO assert !
bool CostSaturation::add_order(std::vector<int> order, int* order_id){
	for(size_t i = 0; i < scp_orders.size(); i++){
	 	vector<int> existing_order = scp_orders[i];
		if(existing_order == order){			
			//cout << " already exists" << endl;
			*order_id = i;
			return true;	
		}
	}
	scp_orders.push_back(order);
	//cout << "ADD ORDER: Number of orders:  " << scp_orders.size() << endl;
	*order_id = scp_orders.size() - 1;
	vector<int> rm_copy = remaining_costs; // check if it works this way
	remaining_costs_order.push_back(rm_copy);
	//cout << "New Remaining Cost add_order: " << *order_id << endl;
	for(Abstraction* abs : abstractions){
		abs->add_cost_partitioning();	
	}
	return false;
}
	
//TODO
void CostSaturation::delete_order(int order_id){
	scp_orders.erase (scp_orders.begin()+order_id);
}
	
	
void CostSaturation::remove_abstraction(int pos){
	/*
	cout << "---------------------------------------------------------" << endl;
	cout << "Delete abstraction at pos: " << pos << endl; 
	cout << "NUMBER OF ABSTRACTIONS: " << abstractions.size() << endl;
	*/
	Abstraction* abs = abstractions[pos];
	abstractions.erase(abstractions.begin() + pos);
	for(size_t o = 0; o < scp_orders.size(); o++){
		//cout << "Order: " << o << endl;
		/*
		cout << "Old Remaining" << endl;
		for(int c : remaining_costs_order[o]){
			cout << c << " ";	
		}
		cout << endl;
		*/
		vector<int> cost = abs->get_costs_partitioning(o);
		/*
		cout << "Cost" << endl;
		for(int c : cost){
			cout << c << " ";	
		}
		cout << endl;
		*/
		
		//The cost of the deleted abstraction is added to the remaining cost
		add_cost_partitioning(&(remaining_costs_order[o]), &cost);
		/*
		cout << "New Remaining" << endl;
		for(int c : remaining_costs_order[o]){
			cout << c << " ";	
		}
		cout << endl;
		cout << ".................................................." << endl;
		*/
	}
	
	//heuristic_functions.erase(heuristic_functions.begin() + pos); TODO can not be delted
	delete abs;
	
	//erase abstraction from all costpartitionings
	for(size_t j = 0; j < scp_orders.size(); j++){		
		for(size_t i = 0; i < scp_orders[j].size(); i++){
			//cout << scp_orders[j][i] << " ";
			if(scp_orders[j][i] == pos){
				scp_orders[j].erase(scp_orders[j].begin() + i);
				break;
			}
		}
		//derease the position in the scp orders of all abstraction behind the removed one
		for(size_t i = 0; i < scp_orders[j].size(); i++){
			if(scp_orders[j][i] > pos){
				scp_orders[j][i]--;
			}
		}
		/*
		cout << endl;
		cout << "New Order: ";
		for(int o : scp_orders[j]){
			cout << o << " ";	
		}
		cout << endl;
		*/
	}
	
}
	
std::vector<std::vector<int>>* CostSaturation::get_unused_cost(){
	return &remaining_costs_order;
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
	
	cout << "Orders: " << endl;
	for(vector<int> o : scp_orders){
		for(int pos : o){
			cout << pos << " ";	
		}
		cout << endl;
	}
	cout << endl;
	/*
	for(Abstraction* abs : abstractions){
		abs->print_cost();	
	}
	*/
	
	
}
	
void CostSaturation::print_statistics_end() const{
	cout << "----- COST PARTITIONING -----" << endl;
	cout << "Number of orders: " << scp_orders.size() << endl;
	cout << "Orders: " << endl;
	for(vector<int> o : scp_orders){
		for(int pos : o){
			cout << pos << " ";	
		}
		cout << endl;
	}
	cout << endl;
	cout << "UNUSED COST: " << endl;
	for(size_t i = 0; i < remaining_costs_order.size(); i++){
		int used = 0;
		//cout << "Remaining Cost: " << i << endl;
		for(int j : remaining_costs_order[i]){
			//cout << j << " ";	
			if(j == 0){
				used++;	
			}
		}
		//cout << endl;
		cout << "Used: " << used << "/" << (remaining_costs_order[i].size()) << "-->" << (((float) used) / remaining_costs_order[i].size()) << endl;
	}
	
	//Compute average abstraction size
	int abs_size_total = 0;
	for(Abstraction* abs : abstractions){
		abs_size_total += abs->get_num_states();
	}
	cout << "Average abstratcion size end: " << (abs_size_total / abstractions.size()) << endl;
}
}
