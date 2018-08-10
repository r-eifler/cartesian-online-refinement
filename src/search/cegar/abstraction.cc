#include "abstraction.h"

#include "abstract_state.h"
#include "utils.h"

#include "../heuristic.h"
#include "../globals.h"
#include "../task_tools.h"
#include "../causal_graph.h"
#include "../task_proxy.h"

#include "../utils/logging.h"
#include "../utils/memory.h"

#include "../tasks/domain_abstracted_task_factory.h"
#include "../tasks/modified_goals_task.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using namespace std;

namespace cegar {
struct Flaw {
    // Last concrete and abstract state reached while tracing solution.
    const State concrete_state;
    // TODO: After conversion to smart pointers, store as unique_ptr?
    AbstractState *current_abstract_state;
    // Hypothetical Cartesian set we would have liked to reach.
    const AbstractState desired_abstract_state;

    Flaw(
        State &&concrete_state,
        AbstractState *current_abstract_state,
        AbstractState &&desired_abstract_state)
        : concrete_state(move(concrete_state)),
          current_abstract_state(current_abstract_state),
          desired_abstract_state(move(desired_abstract_state)) {
    }

    vector<Split> get_possible_splits() const {
        vector<Split> splits;
        /*
          For each fact in the concrete state that is not contained in
          the current abstract state (reason: abstract and concrete
          traces diverged) or the desired abstract state (reason:
          unsatisfied precondition or goal), loop over all values of
          the corresponding variable. The values that are in both the
          current and the desired abstract state are the "wanted" ones.
        */
		
		/*
		cout << "GET POSSIBLE SPLITS" << endl;
		for(uint i = 0; i < concrete_state.size(); i++){
			cout << "v" << i << " = " << concrete_state[i].get_value() << " ";
		}
		cout << endl;
		cout << "current: " << *current_abstract_state << endl;
		cout << "desired: " << desired_abstract_state << endl;
		*/

        for (FactProxy wanted_fact_proxy : concrete_state) {
            FactPair fact = wanted_fact_proxy.get_pair();
            if (!current_abstract_state->contains(fact.var, fact.value) ||
                !desired_abstract_state.contains(fact.var, fact.value)) {
                VariableProxy var = wanted_fact_proxy.get_variable();
                int var_id = var.get_id();
                vector<int> wanted;
                for (int value = 0; value < var.get_domain_size(); ++value) {
                    if (current_abstract_state->contains(var_id, value) &&
                        desired_abstract_state.contains(var_id, value)) {
                        wanted.push_back(value);
                    }
                }
				/*cout << "Wanted: " << endl;
				for(uint i = 0; i < wanted.size(); i++){
					cout << wanted[i] << " ";
				}
				cout << endl;
				*/
				// add random variables
				
				//cout << "Var : " << var_id <<  " domain size: " << var.get_domain_size() << endl;	
				/*
				vector<int> random_values;
				for (int value = 0; value < var.get_domain_size(); ++value) {
                    if (current_abstract_state->contains(var_id, value) && ! desired_abstract_state.contains(var_id, value)) {
						random_values.push_back(value);
					}
				}
				//cout << "candidates: " << random_values.size() << endl;
				std::random_shuffle(random_values.begin(), random_values.end());
				cout << "Random values: " << endl;
				for(uint i = 0; i < random_values.size(); i++){
					cout << random_values[i] << " ";
				}
				cout << endl;
				
				for (uint i = 0; i < random_values.size(); i++) {
					int v = random_values[i];
					//cout << "Next value: " << v <<  " counter: " << i << " size: " << random_values.size() << endl;
					if((int) wanted.size() >= current_abstract_state->count(var_id)/2){
						break;
					}
                    if (current_abstract_state->contains(var_id, v) && ! desired_abstract_state.contains(var_id, v)) {
                        wanted.push_back(v);
                    }
          
				}
				*/
				/*
				cout << "Wanted: " << endl;
				for(uint i = 0; i < wanted.size(); i++){
					cout << wanted[i] << " ";
				}
				cout << endl;
				*/

                //assert(!wanted.empty());
                //TODO !!!!!!!!!!!!!!!!!!!!
                if(wanted.empty()){
                    return splits;   
                }
				//test if variable domain can be split
				if(current_abstract_state->count(var_id) > 1){
					splits.emplace_back(var_id, move(wanted));
				}
            }
        }
        //assert(!splits.empty());
		//TODO !!!!!!!!!!!!!!!!!!!!
        return splits;
    }
};

    
Abstraction::Abstraction(
    shared_ptr<AbstractTask> task,
    shared_ptr<AbstractTask> original_task,
    int max_states,
    int max_non_looping_transitions,
    double max_time,
    bool use_general_costs,
    PickSplit pick,
	bool use_manhatten_distance,
    utils::RandomNumberGenerator &rng,
    bool debug)
    : task(task),
	  original_task(original_task),
      task_proxy(*task),
      max_states(max_states),
      max_non_looping_transitions(max_non_looping_transitions),
      use_general_costs(use_general_costs),
      abstract_search(get_operator_costs(task_proxy), states),
      split_selector(task, pick),
      transition_updater(task_proxy.get_operators()),
      timer(max_time),
      init(nullptr),
      deviations(0),
      unmet_preconditions(0),
      unmet_goals(0),
	  use_manhatten_distance(use_manhatten_distance),
      debug(debug),
      rng(rng){
       
          
    //Stop timer
    refine_timer.stop();
    update_timer.stop();
          
    //Build causal graph
    /*
	const CausalGraph c_graph =  get_causal_graph(&(*task));
	cout << "+++++++++++++++++++ Causal Graph ++++++++++++++++++" << endl;
	c_graph.dump(task_proxy);
	cout << "+++++++++++++++++++ Causal Graph ++++++++++++++++++" << endl;
    */


	/*
     cout << "+++++++++++++++++++ ACTIONS " << task_proxy.get_operators().size() << "++++++++++++++++++" << endl;
	for(uint i = 0; i < task_proxy.get_operators().size(); i++){
		cout << task_proxy.get_operators()[i].get_name() << endl;
		//cout << "		pre v0=" << task_proxy.get_operators()[i].get_preconditions()[0].get_value() << endl;
	}
	*/	
          
     cout << "+++++++++++++++++++ Goals ++++++++++++++++++" << endl;
     GoalsProxy goals = task_proxy.get_goals();
     for(uint i = 0; i < goals.size(); i++){
		cout << goals[i].get_variable().get_name() <<  " = " << goals[i].get_value() << endl;	
	}
    
          
    assert(max_states >= 1);
    g_log << "Start building abstraction." << endl;
    cout << "Maximum number of states: " << max_states << endl;
    cout << "Maximum number of transitions: "
         << max_non_looping_transitions << endl;
    build(rng);
    g_log << "Done building abstraction." << endl;
    cout << "Time for building abstraction: " << timer << endl;

    /* Even if we found a concrete solution, we might have refined in the
       last iteration, so we should update the distances. */
    update_h_and_g_values();
    update_h_and_g_values(0, false);  
    update_h_values_complete_cost();
    print_statistics();
}


Abstraction::~Abstraction() {
    for (AbstractState *state : states)
        delete state;
}

bool Abstraction::is_goal(AbstractState *state) const {
    return goals.count(state) == 1;
}
    
bool Abstraction::is_refine_goal(AbstractState *state) const {
    return refine_goals.count(state) == 1;
}
    
bool Abstraction::satisfies_goal(State state){
    return is_goal_state(task_proxy, state);   
}
    
bool Abstraction::is_abstract_goal(AbstractState* state){
    //cout << "Is Goals ?: " << *state << endl;
    GoalsProxy gp = task_proxy.get_goals();
    //cout << "goal vars: " << gp.size() << endl;
	for(size_t i = 0; i < gp.size(); i++){
        //cout << "var " << gp[i].get_variable().get_id() << " = " << gp[i].get_value() << endl;
		bool contains = state->getDomains().test(gp[i].get_variable().get_id(), gp[i].get_value());
		if(! contains){
			return false;	
		}
	}
    /*
    for(size_t i = 0; i < additional_goals.size(); i++){
        //cout << "var " << additional_goals[i].first << " = " << additional_goals[i].second << endl;
		bool contains = state->getDomains().test(additional_goals[i].first, additional_goals[i].second);
		if(! contains){
			return false;	
		}
	}
    */
	return true;
}

void Abstraction::separate_facts_unreachable_before_goal() {
    assert(goals.size() == 1);
    assert(states.size() == 1);
    assert(task_proxy.get_goals().size() == 1);
    FactProxy goal = task_proxy.get_goals()[0];
    unordered_set<FactProxy> reachable_facts = get_relaxed_possible_before(
        task_proxy, goal);
    for (VariableProxy var : task_proxy.get_variables()) {
        if (!may_keep_refining())
            break;
        int var_id = var.get_id();
        vector<int> unreachable_values;
        for (int value = 0; value < var.get_domain_size(); ++value) {
            FactProxy fact = var.get_fact(value);
            if (reachable_facts.count(fact) == 0)
                unreachable_values.push_back(value);
        }
        if (!unreachable_values.empty())
            refine(init, var_id, unreachable_values);
    }
    goals.clear();
    goals.insert(states.begin(), states.end());
}

void Abstraction::create_trivial_abstraction() {
    init = AbstractState::get_trivial_abstract_state(
        task_proxy, refinement_hierarchy.get_root());
    transition_updater.add_loops_to_trivial_abstract_state(init);
    goals.insert(init);
    states.insert(init);
}

bool Abstraction::may_keep_refining() const {
    /* TODO: Think about whether we really want to go to the memory limit.
       Without doing so, the algorithm would be more deterministic. */
    return utils::extra_memory_padding_is_reserved() &&
           get_num_states() < max_states &&
           transition_updater.get_num_non_loops() < max_non_looping_transitions &&
           !timer.is_expired();
}

void Abstraction::build(utils::RandomNumberGenerator &rng) {
    create_trivial_abstraction();
    /*
      For landmark tasks we have to map all states in which the
      landmark might have been achieved to arbitrary abstract goal
      states. For the other types of subtasks our method won't find
      unreachable facts, but calling it unconditionally for subtasks
      with one goal doesn't hurt and simplifies the implementation.
    */
    
    if (task_proxy.get_goals().size() == 1) {
		//cout << "...... separate_facts_unreachable_before_goal ... " << endl;
        //separate_facts_unreachable_before_goal();
		//cout << "...... separate_facts_unreachable_before_goal ... " << endl;
    }
	//cout << "...... REFINE ... " << endl;
    bool found_concrete_solution = false;
	//Split all goal facts 
	
	while (may_keep_refining()) {
		AbstractState *abstract_state = *goals.begin(); 
		//cout << "Abstract GOAL State: h=" << abstract_state->get_h_values()[0] << endl << *abstract_state << endl;


		vector<Split> split_facts;
		//cout << "Split values: " << endl;
		for(FactProxy goal : task_proxy.get_goals()){
			//if(state[i] != preState[i]){
			if(abstract_state->count(goal.get_variable().get_id()) > 1){
				//cout << "v" << goal.get_variable().get_id() << " = " << goal.get_value() << endl;
				vector<int> values;
				values.push_back(goal.get_value());
				split_facts.emplace_back(goal.get_variable().get_id(), move(values));
			}
			//}
		}

		if(split_facts.empty()){
			//cout << "NO SPLITS" << endl;
			break;
		}

		const Split &split = split_selector.pick_split(*abstract_state, split_facts, rng);

		if(abstract_state->count(split.var_id) > 1){
				refine(abstract_state, split.var_id, split.values);  
				//AbstractState *abstract_state = *goals.begin(); 
				//cout << "Abstract GOAL State: h=" << abstract_state->get_h_values()[0] << endl << *abstract_state << endl;
		}
		//cout << "---------------------------------------------------------------------------" << endl;
    }

	//print_states();
	//Split the variable which indicates the position completely

	if(use_manhatten_distance){
	cout << "--------------------------------------------------------- Manhatten Distance ------------------------------------------------------" << endl;

	AbstractState* state_to_split = init;
	int var_pos = 0;
	int domain_size = state_to_split->count(0);
	for(int i = 0; i < state_to_split->getDomains().size(); i++){
		int d = state_to_split->count(i);
		if(d > domain_size){
			var_pos = i;
			domain_size = d;
		}
	}
	cout << "Cell variable has position: " << var_pos << " with domain size: " << domain_size << endl;
	
	int split_value = 0;
    while (may_keep_refining() && state_to_split->count(var_pos) > 1) {
		//cout << "State to split" << *state_to_split << endl;
	
		//cout << "Split var: " << var_pos << " = " << split_value << endl; 
		vector<int> values;
		values.push_back(split_value++);
		if(! state_to_split->contains(var_pos, values[0])){
			continue;
		}

		pair<AbstractState*, AbstractState*> result_states = refine(state_to_split, var_pos, values);  
		state_to_split = result_states.first;
		//cout << "Domain size: " << state_to_split->count(var_pos) << endl;
		//cout << "may_keep_refining: " << may_keep_refining() << endl;
	}
	return;
	}

    while (may_keep_refining()) {
		
        //cout << "-----------------------" << endl;
        bool found_abstract_solution = abstract_search.find_solution(init, goals);
        if (!found_abstract_solution) {
            //cout << "Abstract problem is unsolvable!" << endl;
            break;
        }
        unique_ptr<Flaw> flaw = find_flaw(abstract_search.get_solution(), init, task_proxy.get_initial_state());
        if (!flaw) {
            found_concrete_solution = true;
            break;
        }
        AbstractState *abstract_state = flaw->current_abstract_state;
		//cout << "SPLIT: " << *abstract_state << endl;

        vector<Split> splits = flaw->get_possible_splits();
        const Split &split = split_selector.pick_split(*abstract_state, splits, rng);
        refine(abstract_state, split.var_id, split.values);
        //cout << "-----------------------" << endl;
		

		//print_states();
    }
	//print_states();
    cout << "Concrete solution found: " << found_concrete_solution << endl;
}
    
Node* Abstraction::get_node(const State &state) const {
    return refinement_hierarchy.get_node(state);   
}
    
const TaskProxy* Abstraction::get_Task() {
    return &task_proxy;   
}
    
void Abstraction::update_h_values(){
    update_h_and_g_values();
}
    
void Abstraction::update_Task(shared_ptr<AbstractTask> abstask){
    task = abstask;   
    task_proxy = TaskProxy(*abstask);
    abstract_search.update_operator_costs(get_operator_costs(task_proxy));
}
    
std::shared_ptr<AbstractTask> Abstraction::get_AbsTask(){
   return task;     
}

std::shared_ptr<AbstractTask> Abstraction::get_OriginalAbsTask(){
   return original_task;     
}
    

int Abstraction::refineBasedOnBellman(const State &state, const State &minSucc){

	//cout << "********** refine based on bellman *******************" << endl;
	
	/*
	cout << "Current State: " << endl;
	state.dump_pddl();
	state.dump_fdr();
	cout << endl;

	cout << "MinSucc State: " << endl;
	minSucc.dump_pddl();
	minSucc.dump_fdr();
	cout << endl;
	*/
	
	AbstractState *abstract_state = get_node(state)->get_AbstractState(); 
	//cout << "Abstract State: h=" << abstract_state->get_h_values()[0] << endl << *abstract_state << endl;

	AbstractState *abstract_minSucc = get_node(minSucc)->get_AbstractState(); 
	//cout << "Abstract pre State: h=" << abstract_minSucc->get_h_values()[0] << endl << *abstract_minSucc << endl;

	//If both states are in the same abstract state:
	//split it such that it is not the case anymore
	if(abstract_minSucc == abstract_state){
		num_same_abstract_state++;
		//cout << "SAME ABSTRACT STATE" << endl;

		//TODO split multiple times ?
		vector<Split> split_facts;
		//cout << "Split values: " << endl;
		for(uint i = 0; i < state.size(); i++){
			if(state[i] != minSucc[i]){
				if(abstract_state->count(i) > 1){
					//cout << "v" << i << " = " << preState[i].get_value() << endl;;
					vector<int> values;
					values.push_back(minSucc[i].get_value());
					split_facts.emplace_back(i, move(values));
				}
			}
		}

		if(split_facts.empty()){
			//cout << "NO SPLITS" << endl;
			return 0;
		}

		const Split &split = split_selector.pick_split(*abstract_state, split_facts, rng);

		if(abstract_state->count(split.var_id) > 1){
				refine(abstract_state, split.var_id, split.values);  
				//print_states();
				return 1;
		}

		return 0;
	}


	//Abstract states are not the same
	vector<State> no_other_goals;
	return onlineRefine(state, no_other_goals, 1, std::numeric_limits<int>::max(), NULL);

}


int Abstraction::refineBellmanStyle(const State & state){
	
	cout << "------> refine bellman style" << endl;
	AbstractState *abstract_state = NULL;
	if(states.size() == 1){
		abstract_state = *(states.begin());	
	}
	else{
		abstract_state = get_node(state)->get_AbstractState(); 
	}

	bool found_abstract_solution = abstract_search.find_solution(abstract_state, goals);

	if (!found_abstract_solution) {
		cout << "Abstract problem is unsolvable!" << endl;
		return 0;
	}

	Solution sol = abstract_search.get_solution();
	cout << "Solution lenght: " << sol.size() << endl;
	for(uint i = 0; i < sol.size() ; i++){
		OperatorProxy op = task_proxy.get_operators()[sol[i].op_id];
		cout << "Shortcut : " << op.get_name()<< endl;
	}

	unique_ptr<Flaw> flaw = find_flaw_bellman(abstract_search.get_solution(), abstract_state, state); 

	vector<Split> splits = flaw->get_possible_splits();
	if(splits.empty()){
		cout << "Split empty" << endl;
		return 0;
	}
   
	const Split &split = split_selector.pick_split(*abstract_state, splits, rng);

	cout << "Split: ";
	cout << *abstract_state;
	cout << " with : " << split.var_id << " = {" ;
	for(int v : split.values){
			cout << v << " ";
	}
	cout << "}" << endl;

	refine(abstract_state, split.var_id, split.values);  

	//print_states();
	return 1;
}


std::unique_ptr<Flaw> Abstraction::find_flaw_bellman(const Solution &solution, AbstractState *start_state, State concrete_start_state){

	OperatorProxy op = task_proxy.get_operators()[solution.begin()->op_id];
	cout << "Precondition: " << endl;
	for(uint i = 0; i < op.get_preconditions().size(); i++){
		cout << "v" << op.get_preconditions()[i].get_variable().get_id() << " = " << op.get_preconditions()[i].get_value() << endl; 
	}
    return utils::make_unique_ptr<Flaw>(move(concrete_start_state), start_state,
        AbstractState::get_abstract_state(task_proxy, op.get_preconditions()));

}


int Abstraction::refineSplitPre(const State &state, const State preState){
	/*
	cout << "---> Refine pre state" << endl;
	cout << "Current State: " << endl;
	state.dump_pddl();
	state.dump_fdr();
	cout << endl;

	cout << "Pre State: " << endl;
	preState.dump_pddl();
	preState.dump_fdr();
	cout << endl;
	*/

	AbstractState *abstract_state = get_node(state)->get_AbstractState(); 
	//cout << "Abstract State: h=" << abstract_state->get_h_values()[0] << endl << *abstract_state << endl;

	AbstractState *abstract_pre_state = get_node(preState)->get_AbstractState(); 
	//cout << "Abstract pre State: h=" << abstract_pre_state->get_h_values()[0] << endl << *abstract_pre_state << endl;

	//If both states are in the same abstract state:
	//split it such that it is not the case anymore
	if(abstract_pre_state == abstract_state){
		num_same_abstract_state++;
		cout << "SAME ABSTRACT STATE" << endl;

		//TODO split multiple times ?
		vector<Split> split_facts;
		//cout << "Split values: " << endl;
		for(uint i = 0; i < state.size(); i++){
			if(state[i] != preState[i]){
				if(abstract_state->count(i) > 1){
					//cout << "v" << i << " = " << preState[i].get_value() << endl;;
					vector<int> values;
					values.push_back(preState[i].get_value());
					split_facts.emplace_back(i, move(values));
				}
			}
		}

		if(split_facts.empty()){
			//cout << "NO SPLITS" << endl;
			return 0;
		}

		const Split &split = split_selector.pick_split(*abstract_state, split_facts, rng);

		if(abstract_state->count(split.var_id) > 1){
				refine(abstract_state, split.var_id, split.values);  
		}
	}
	else{
		//abstract states are not the same
		abstract_search.find_solution(abstract_state, goals);
		Solution solution_state = abstract_search.get_solution();

		abstract_search.find_solution(abstract_pre_state, goals);
		Solution solution_pre_state = abstract_search.get_solution();

		//Find meeting point
		//collect all states on the solution of state s'
		unordered_set<AbstractState*> solution_states;
		solution_states.insert(abstract_state);
		for (const Transition &step : solution_state) {
			solution_states.insert(step.target);
		}
	
		//Find the first state the solution of s which is also in the solution
		//of s'
		AbstractState* meeting_point = NULL; 
		int op_id_pre = 0;
		for(const Transition & step : solution_pre_state){
			if(solution_states.find(step.target) != solution_states.end()){
				op_id_pre = step.op_id;
				meeting_point = step.target;
			}
		}
		//cout << "MEETING POINT = " << *meeting_point << endl;
		if(is_abstract_goal(meeting_point)){
			cout << "GOAL IS MEETING POINT" << endl;
			num_spurious_meeting_point_goal++;
			vector<State> new_goals;
			int r = onlineRefine(state, new_goals, 1, 1, NULL); 	
			//print_states();
			return r;
		}

		//TODO find matching transition in solution od s'
		
		int op_id = 0;
		for (const Transition &step : solution_state) {
			if(step.target == meeting_point){
				op_id = step.op_id;
			}
		}

		//is spurous
		//
		OperatorProxy op_pre = task_proxy.get_operators()[op_id_pre];
		EffectsProxy effects_pre = op_pre.get_effects();
		OperatorProxy op = task_proxy.get_operators()[op_id];
		EffectsProxy effects = op.get_effects();

		int is_spurous_meeting_point = false;
		for(uint i = 0; i < effects.size(); i++){
			int var = effects[i].get_fact().get_variable().get_id();
			int value = effects[i].get_fact().get_value();
			for(uint j = 0; j < effects_pre.size(); j++){
				if(effects_pre[j].get_fact().get_variable().get_id() == var && 
						effects_pre[j].get_fact().get_value() == value){
					break;
				}
				is_spurous_meeting_point = true;
				goto end_comparison;
			}
		}

		end_comparison:
		if(is_spurous_meeting_point){
			num_spurious_meeting_point++;
			//Split according to the effect 
			cout << "SPURIOUS MEETING POINT" << endl;
			vector<Split> split_facts;
			//cout << "Effect: " << endl;
			for(uint i = 0; i < effects.size(); i++){
				int var = effects[i].get_fact().get_variable().get_id();
				int value = effects[i].get_fact().get_value();
				//cout << "Domain size var " << var << ": " << meeting_point->count(var) << endl;
				if(meeting_point->count(var) > 1){
					//cout << "v" << var << " = " << value << endl;;
					vector<int> values;
					values.push_back(value);
					split_facts.emplace_back(var, move(values));
				}
			}

			if(split_facts.empty()){
				//cout << "NO SPLITS" << endl;
				return 0;
			}

			const Split &split = split_selector.pick_split(*meeting_point, split_facts, rng);
			//cout << "Split: var" << split.var_id << " = " << split.values[0] << endl; 

			if(meeting_point->count(split.var_id) > 1){
					refine(meeting_point, split.var_id, split.values);  
			}

		}
		else{
			num_standard_refine++;
			vector<State> new_goals;
			onlineRefine(state, new_goals, 1, 1, NULL); 	
		}
	}
	//print_states();
	//cout << "Abstraction size: " << states.size() << endl;
	return 1;
}


int Abstraction::refineSplitOnCondition(const State &state, const State &preState, const std::vector<std::pair<int,int>> conditions){


	cout << "Current State: " << endl;
	state.dump_pddl();
	state.dump_fdr();
	AbstractState *abstract_current_state = get_node(state)->get_AbstractState(); 
	cout << *abstract_current_state << endl;
	cout << endl;

	cout << "Pre State: " << endl;
	preState.dump_pddl();
	preState.dump_fdr();
	AbstractState *abstract_pre_state = get_node(preState)->get_AbstractState(); 
	cout << *abstract_pre_state << endl;
	cout << endl;


	cout << "Precondition: ";
	for(uint i = 0; i < conditions.size(); i++){
		cout << "v" << conditions[i].first << " = " << conditions[i].second << " ";
	}
	cout << endl;

	unique_ptr<Flaw> flaw = find_flaw_Condition(conditions, abstract_current_state, state);

	vector<Split> splits = flaw->get_possible_splits();
	if(splits.empty()){
		cout << "Split empty" << endl;

		unique_ptr<Flaw> flaw = find_flaw_Condition(conditions, abstract_pre_state, state);
		vector<Split> splits = flaw->get_possible_splits();
		if(splits.empty()){
			cout << "Split empty" << endl;
			vector<State> new_goals;
			//onlineRefine(state, new_goals, 1, 10000000, NULL);
			return 0;
		}

		const Split &split = split_selector.pick_split(*abstract_pre_state, splits, rng);
		cout << "Split: v" << split.var_id << " = " << split.values[0] << endl;

		if(abstract_pre_state->count(split.var_id) > 1){
			refine(abstract_pre_state, split.var_id, split.values);  
		}

		//print_states();
		return 1;
	}
   
	const Split &split = split_selector.pick_split(*abstract_current_state, splits, rng);

	if(abstract_current_state->count(split.var_id) > 1){
		refine(abstract_current_state, split.var_id, split.values);  
	}

	//print_states();
	return 1;
}

std::unique_ptr<Flaw> Abstraction::find_flaw_Condition(const std::vector<std::pair<int,int>> conditions, AbstractState *start_state, State concrete_start_state){

    return utils::make_unique_ptr<Flaw>(move(concrete_start_state), start_state,
        AbstractState::get_abstract_state_Condition(task_proxy, conditions));
}

int Abstraction::refineNewGoal(const State &state, const State &new_goal){
	int num_of_Iter = 1;
    refined = false;
    if(!(utils::extra_memory_padding_is_reserved())){
     return 0;   
    }

	refine_goals.clear();
	AbstractState* abs_goal = refinement_hierarchy.get_node(new_goal)->get_AbstractState();
	refine_goals.insert(abs_goal);

    refinement_calls++;
    int refined_states = 0;
    refine_timer.resume();
	while((utils::extra_memory_padding_is_reserved() && num_of_Iter > 0)){
        AbstractState *start_state = NULL;
		if(states.size() == 1){
			start_state = *(states.begin());	
		}
		else{
			start_state = get_node(state)->get_AbstractState(); 
		}
		
		bool found_abstract_solution  = abstract_search.find_solution(start_state, refine_goals);
        if (!found_abstract_solution) {
            //cout << "Abstract problem is unsolvable!" << endl;
            break;
        }

       //new goal facts
	   //cout << "New Goal facts: " << endl;
		vector<pair<int,vector<int>>> goal_facts_split;
		for(uint i = 0; i < new_goal.size(); i++){
			vector<int> values;
			values.push_back(new_goal[i].get_value());
			//cout << "v" << i << " = " << new_goal[i].get_value() << endl;
			goal_facts_split.push_back(make_pair(i, values));
		}


        //Find flaw starting from the spesified state "start_state"
        unique_ptr<Flaw> flaw = find_flaw_online(abstract_search.get_solution(), start_state, state, goal_facts_split);
        if (!flaw) {
            //TODO solution found
            //cout << "NO FLAW FOUND" << endl;
            break;
        }
        AbstractState *abstract_state = flaw->current_abstract_state;
        vector<Split> splits = flaw->get_possible_splits();
        if(splits.empty()){
            //cout << "Split empty" << endl;
            refine_timer.stop();
            refined = refined_states > 0;
            return refined_states;
        }
       
        const Split &split = split_selector.pick_split(*abstract_state, splits, rng);

		/*
		cout << "Split: ";
		cout << *abstract_state;
		cout << " on : " << split.var_id << " = {" ;
		for(int v : split.values){
				cout << v << " ";
		}
		cout << "}" << endl;
		*/
	
		refine(abstract_state, split.var_id, split.values);  
		refined_states++;			
    	
		num_of_Iter--;
   }
    refine_timer.stop();
    
    refined = refined_states > 0;

	//print_states();
    return refined_states;

}
    
int Abstraction::onlineRefine(const State &state, std::vector<State> new_goals, int num_of_Iter, int max_states_refine, std::vector<std::vector<int>> *unused_cost){
	//cout << "******************************** NORMAL ONLINE REFINE **********************************" << endl;
    refined = false;
    int state_border = get_num_states() + max_states_refine < 0 ? max_states_refine : get_num_states() + max_states_refine;
    if(!(utils::extra_memory_padding_is_reserved() && get_num_states() < state_border && num_of_Iter >= 0)){
     return 0;   
    }

	refine_goals.clear();
	vector<pair<int,vector<int>>> goal_facts_split;
	if(new_goals.size() > 0){
		State new_goal = new_goals[0];
		refine_goals.insert(get_node(new_goal)->get_AbstractState());
		for(uint i = 0; i < new_goal.size(); i++){
			vector<int> values;
			values.push_back(new_goal[i].get_value());
			//cout << "v" << i << " = " << new_goal[i].get_value() << endl;
			goal_facts_split.push_back(make_pair(i, values));
		}
	}

	refinement_calls++;
    int refined_states = 0;
    refine_timer.resume();
	//cout << "while condition: " << endl << "\tPadding: " << utils::extra_memory_padding_is_reserved() 
	//	<< endl << "\tState border: " << (get_num_states() < state_border) << endl << "\tIter" << (num_of_Iter > 0) << endl;
	while((utils::extra_memory_padding_is_reserved() && get_num_states() < state_border && num_of_Iter > 0)){
        AbstractState *start_state = NULL;
		if(states.size() == 1){
			start_state = *(states.begin());	
		}
		else{
			start_state = get_node(state)->get_AbstractState(); 
		}
		
        //bool found_abstract_solution = abstract_search.find_solution(start_state, goals);
	
		bool found_abstract_solution = false;
		if(refine_goals.size() > 0){
			//cout << "Use frontier as goals: " << refine_goals.size() << endl;
			found_abstract_solution = abstract_search.find_solution(start_state, refine_goals);
		}
		else{
			//cout << "use original goals: " << endl;
			found_abstract_solution = abstract_search.find_solution(start_state, goals);
		}

        if (!found_abstract_solution) {
            //cout << "Abstract problem is unsolvable!" << endl;
            break;
        }

        
        //Find flaw starting from the spesified state "start_state"
        unique_ptr<Flaw> flaw;
		if(refine_goals.size() > 0){
			flaw = find_flaw_online(abstract_search.get_solution(), start_state, state, goal_facts_split);
		}
		else{
			flaw = find_flaw(abstract_search.get_solution(), start_state, state);
		}

        if (!flaw) {
            //TODO solution found
            //cout << "No flaw found" << endl;
            break;
        }
        AbstractState *abstract_state = flaw->current_abstract_state;
        vector<Split> splits = flaw->get_possible_splits();
        if(splits.empty()){
            //cout << "Split empty" << endl;
            refine_timer.stop();
            refined = refined_states > 0;
            return refined_states;
        }
       
        const Split &split = split_selector.pick_split(*abstract_state, splits, rng);

		/*
		cout << "Split: ";
		cout << *abstract_state;
		cout << " with : " << split.var_id << " = {" ;
		for(int v : split.values){
				cout << v << " ";
		}
		cout << "}" << endl;
		*/
	
		bool split_useful = split_usefull(abstract_state, split.var_id, split.values, unused_cost);
		if(split_useful){	
			usefull_splits++;	 
			//cout << "refine" << endl;
			refine(abstract_state, split.var_id, split.values);  
			refined_states++;			
		}
		else{
			break;	
		}
    	
		num_of_Iter--;
   }
    refine_timer.stop();
    
    refined = refined_states > 0;

	//print_states();
    return refined_states;
}
	
bool Abstraction::split_usefull(AbstractState * state, int var, const std::vector<int> &wanted, std::vector<std::vector<int>> *unused_cost){
	if(unused_cost == NULL){
		return true;	
	}
	
	
	pair<AbstractState *, AbstractState *> new_states = state->split_testwise(var, wanted);
    AbstractState *v1 = new_states.first;
    AbstractState *v2 = new_states.second;

    /*
    cout << "-----------------------" << endl;
    cout << "Split " << *state << " h = " << state->get_h_value() << " in " << endl;
    cout << *v1 << " h = " << v1->get_h_value() << " and " << endl;
    cout << *v2 << " h = " << v2->get_h_value() << endl;
    cout << "-----------------------" << endl;
    */
	
    pair<vector<int>, vector<int>> trans = transition_updater.loops_to_transition(state, v1, v2, var);
	/*
	cout << "----------------------------------------------------------------------------------" << endl;
	for(vector<int> cost : (*unused_cost)){
		for(int c : cost){
			cout << c << " ";	
		}
		cout << endl;
	}
	cout << "v1tov2: ";
	for(int op_id : trans.first){
		cout << op_id << " ";
	}
	cout << endl;
	cout << "v2tov1: ";
	for(int op_id : trans.second){
		cout << op_id << " ";
	}
	cout << endl;
	*/
	//Check if all transitions from v1 to v2 (respectively v2 to v1) are contained in any unused_cost
	//1. v1 to v2
	bool v1tov2_has_cost = false;
	for(size_t i = 0; i < unused_cost->size(); i++){
		bool has_cost = true;
		for(int op_id : trans.first){
			if((*unused_cost)[i][op_id] == 0 && costs_partitionings[i][op_id] == 0){
				has_cost = false;
				break;
			}
		}
		if(has_cost){
			v1tov2_has_cost = true;
			break;
		}
	}
	
	bool v2tov1_has_cost = false;
	if(!v1tov2_has_cost){
		for(size_t i = 0; i < unused_cost->size(); i++){
			bool has_cost = true;
			for(int op_id : trans.second){
				if((*unused_cost)[i][op_id] == 0 && costs_partitionings[i][op_id] == 0){
					has_cost = false;
					break;
				}
			}
			if(has_cost){
				v2tov1_has_cost = true;
				break;
			}
		}
	}
	//cout << "---> Refine usefull: " << (v1tov2_has_cost || v2tov1_has_cost) << endl;
	return v1tov2_has_cost || v2tov1_has_cost;
}
	
void Abstraction::addGoals(GoalsProxy goalsProxy){
	vector<FactPair> goal_facts = get_fact_pairs(goalsProxy);
	
	task = make_shared<extra_tasks::ModifiedGoalsTask>(task, move(goal_facts));    
    task_proxy = TaskProxy(*task);
	
	AbstractStates newGoals;
	for(AbstractState* state: goals){
		if(is_abstract_goal(state)){
			newGoals.insert(state);	
		}
	}
	goals = newGoals;
}
    
bool Abstraction::merge(Abstraction* abs){

    refined = false;
    //Check if goals allow merge
    GoalsProxy goalsAbs1 = task_proxy.get_goals();
    GoalsProxy goalsAbs2 = abs->get_Task()->get_goals();
    //cout << "Abs 2 goal vars: " << goalsAbs2.size() << endl;
	for(size_t i = 0; i < goalsAbs2.size(); i++){
        //cout << "var " << goalsAbs2[i].get_variable().get_id() << " = " << goalsAbs2[i].get_value() << endl;
        for(size_t j = 0; j < goalsAbs1.size(); j++){
            if(goalsAbs2[i].get_variable().get_id() == goalsAbs1[j].get_variable().get_id() 
               && goalsAbs2[i].get_value() != goalsAbs1[j].get_value()){
                return false;   
            }
        }
	}
    //Update task with new goals
    
    vector<FactPair> goal_facts_original = get_fact_pairs(task_proxy.get_goals());
    /*
    cout << "..............." << endl;
    cout << "Original goal facts: " << endl;
    for(FactPair fp : goal_facts_original){
           cout << fp.var << " = " << fp.value << endl;
    }
    print_states();
    //print_cost();
    */
    
    const TaskProxy *task_proxy_goals =  abs->get_Task();
    vector<FactPair> goal_facts_add = get_fact_pairs(task_proxy_goals->get_goals());
    
	/*
     cout << "..............." << endl;
        cout << "Additional goal facts: " << endl;
    for(FactPair fp : goal_facts_add){
           cout << fp.var << " = " << fp.value << endl;
    }
    abs->print_states();
    //abs->print_cost();
    */
    
    goal_facts_original.insert(goal_facts_original.end(), goal_facts_add.begin(), goal_facts_add.end());
    task = make_shared<extra_tasks::ModifiedGoalsTask>(task, move(goal_facts_original));
    
    task_proxy = TaskProxy(*task);

    /*
    //cout << "---------- Check existing goals ---------" << endl;
    AbstractStates absGoals;
    for(AbstractState* a : goals){
        bool is_goal = is_abstract_goal(a);
        if(is_goal){              
            //cout << "Keep goal state: " << *a << endl;
            absGoals.insert(a);
        }
        else{
            //cout << "Removed goal state: " << *a << endl;
        }   
    }
    goals = absGoals;
    //cout << "---------- Check existing goals ---------" << endl; 
	*/
	
    unordered_set<pair<AbstractState*, Node*>> states_to_split;
    //init all original states with the root node of the refinement hierarchy
    //cout << "--------- init states_to_split --------------" << endl;
    for(AbstractState* state : states){  
        //cout << *state << endl;
        states_to_split.insert(make_pair(state, abs->refinement_hierarchy.get_root()));
    }
    //cout << "---------------------------------------------" << endl;
    
    while(states_to_split.size() > 0){
        /*
        cout << "States TODO: " << endl;
        for(pair<AbstractState*, Node*> state : states_to_split){  
            cout << *(state.first) << endl;        
        }
        cout << endl;
        */
        pair<AbstractState*, Node*> next = *(states_to_split.begin());
        states_to_split.erase(states_to_split.begin());
		//cout << "---------------------------------------------" << endl;
        //cout << "Next state: " << *(next.first) << endl;

        Node* lc = NULL;
        Node* rc = NULL;
        pair<int, vector<int>> split = next.second->get_wanted_vars(next.first, &lc, &rc);
        if(rc != NULL){
				/*
               cout << "Split on: var " << split.first << " = {";
				for(int v : split.second){
					cout << v << " ";
				}
				cout << "}" << endl;
				*/
               
               pair<AbstractState*, AbstractState*> result = refineMerge(next.first, split.first, split.second);
				if(is_goal(result.second)){
				   for(AbstractState* g2 : abs->goals){
						if(g2->is_more_general_than(*(result.first))){
							bool subset = false;
							for(AbstractState* g1 : goals){
								if(g1->is_more_general_than(*(result.first))){
									subset = true;
									goals.insert(result.first);	
									//cout << "Could get more goals based on: " << *(result.first) << endl;
									break;
								}
						   	}
							if(subset){
								break;	
							}
						}
				   }
				}
			
               /*
               cout << "Result state: " << endl;
               cout << "    " << *(result.first) << endl;
               cout << "    " << *(result.second) << endl;
               */
               states_to_split.insert(make_pair(result.first, lc));
               states_to_split.insert(make_pair(result.second, rc));       
               
        }
        
        //cout << "---------------------------------------------" << endl;
    }
	/*
    cout << "++++++++++++ RESULT MERGE +++++++++++++++++++++" << endl;
    print_states();
    //print_cost();
	*/
    
    
    //update_h_and_g_values();
    refined = true;
    return true;
}
	
std::pair<AbstractState*, AbstractState*> Abstraction::refineMerge(AbstractState *state, int var, const std::vector<int> &wanted){
	if (debug){
		cout << "---------------------------------------" << endl;
        cout << "MergeRefine " << *state << " for " << var << "=" << wanted << endl;
	}
    
    pair<AbstractState *, AbstractState *> new_states = state->split(var, wanted);
    AbstractState *v1 = new_states.first;
    AbstractState *v2 = new_states.second;

    /*
    cout << "-----------------------" << endl;
    cout << "Split " << *state << " h = " << state->get_h_value() << " in " << endl;
    cout << *v1 << " h = " << v1->get_h_value() << " and " << endl;
    cout << *v2 << " h = " << v2->get_h_value() << endl;
    cout << "-----------------------" << endl;
    */
    transition_updater.rewire(state, v1, v2, var);

    states.erase(state);
    states.insert(v1);
    states.insert(v2);

    /* Since the search is always started from the abstract initial state, v2
       is never the new initial state and v1 is never a goal state. */
	//The start state does not matter
    if (state == init) {
        assert(v1->includes(task_proxy.get_initial_state()) || v2->includes(task_proxy.get_initial_state()));
        if(v1->includes(task_proxy.get_initial_state())){
            init = v1;   
            //cout << "new Init 1" << endl;
        }
        else{
            init = v2;
            //cout << "new Init 1" << endl;
        }
		
        if (debug)
            cout << "New init state: " << *init << endl;
    }
	if(is_goal(state)){
		goals.erase(state);
		goals.insert(v2);  
		if (debug)
		cout << "New/additional goal state: " << *v2 << endl;
	}
    int num_states = get_num_states();
    if (num_states % 100000 == 0) {
        g_log << num_states << "/" << max_states << " states, "
              << transition_updater.get_num_non_loops() << "/"
              << max_non_looping_transitions << " transitions" << endl;
    }

    delete state;
    //cout << "RESULT STATE " << *v1 << endl;
    //cout << "RESULT STATE " << *v2 << endl;
    
    return make_pair(v1,v2);
	
}
	
bool Abstraction::are_plans_compatible(Abstraction* abs){
	//current solution
	bool found_abstract_solution = abstract_search.find_solution(init, goals);
	if(!found_abstract_solution){
		return false;	
	}
	//to check solution
	bool found_abstract_solution_check = abs->abstract_search.find_solution(abs->init, abs->goals);
	if(!found_abstract_solution_check){
		return false;	
	}
	
	//check if the two solution are compatible
	Solution solution1 = abstract_search.get_solution();
	Solution solution2 = abs->abstract_search.get_solution();
	/*
	cout << "Solutions: " << endl;
	for(Transition step : solution1){
		cout << step.op_id << " ";	
	}
	cout << endl;
	for(Transition step : solution2){
		cout << step.op_id << " ";	
	}
	cout << endl;
	*/
	
	AbstractState *current_state1 = init;
	AbstractState *current_state2 = abs->init;
	
	Solution::iterator sol1_it = solution1.begin();
	Solution::iterator sol2_it = solution2.begin();
	
	bool error = false;
	while(sol2_it != solution2.end()) {
		if(false){
			cout << "---------------------------------------" << endl;
			cout << "Current States: " << endl;
			cout << "STATE 1: " << *current_state1 << endl;
			cout << "STATE 2: " << *current_state2 << endl;
		}
		//if the action in plan2 is a self loop in abstraction 1 the action can be skipt
		// as it does not influence the solution of abstraction 1
		vector<int> loops = current_state1->get_loops();
		/*
		cout << "loops: ";
		for(int o : loops){
			cout << o << " ";	
		}
		cout << endl;
		*/
		if(find(loops.begin(), loops.end(), sol2_it->op_id) != loops.end()){
			//cout << "---> self loop: " << sol2_it->op_id << endl;
			current_state2 = sol2_it->target;
			sol2_it++;
			continue;
		}
		
		//TODO right position?
		if(sol1_it == solution1.end()){
			//cout << "error" << endl;
			error = true;
			break;
		}
		
		//check if the actions in sol1 and sol2 are the same
		if(sol1_it->op_id == sol2_it->op_id){
			//cout << "---> op: " << sol2_it->op_id << " in both" << endl;
			//if they are the same execute both
			current_state1 = sol1_it->target;
			current_state2 = sol2_it->target;
			sol1_it++;
			sol2_it++;
			continue;
		}
		else{
			//actions are not the same
			//Check if their is an alternative action in abs1 (same action and same target as in plan2)
			bool alternative_found = false;
			//cout << "Alternative Actions: " << endl;
			for(Transition trans : current_state1->get_outgoing_transitions()){
				if(trans.target == sol1_it->target && trans.op_id == sol2_it->op_id){
					//cout << "--> alternative op: " << trans.op_id << " to " << sol1_it->op_id << endl;
					alternative_found = true;
					break;
				}
			}
			//if alternative has been found execute them 
			if(alternative_found){
				current_state1 = sol1_it->target;
				current_state2 = sol2_it->target;
				sol1_it++;
				sol2_it++;
				continue;	
			}
			else{
				//if there is no alternative execute the action in plan 1 only
				if(sol1_it == solution1.end()){
					//cout << "error" << endl;
					error = true;
					break;
				}
				//cout << "---> next action in plan1 op: " << sol1_it->op_id << endl;
				current_state1 = sol1_it->target;
				sol1_it++;
			}
		}
		
	}
	
	//cout << "---------------------------------------" << endl;
	//cout << "plans compatible: " << !error << endl;
	
	return !error;
}



std::pair<AbstractState*, AbstractState*> Abstraction::refine(AbstractState *state, int var, const vector<int> &wanted) {
    if (false){
		cout << "---------------------------------------" << endl;
        cout << "Refine " << *state << " for " << var << "=" << wanted << endl;
	}
    
    pair<AbstractState *, AbstractState *> new_states = state->split(var, wanted);
    AbstractState *v1 = new_states.first;
    AbstractState *v2 = new_states.second;

   
	/*
    cout << "-----------------------" << endl;
    cout << "Split " << *state <<  " in " << endl;
    cout << *v1  << " and " << endl;
    cout << *v2 << endl;
    cout << "-----------------------" << endl;
	*/

    transition_updater.rewire(state, v1, v2, var);

    states.erase(state);
    states.insert(v1);
    states.insert(v2);

    /* Since the search is always started from the abstract initial state, v2
       is never the new initial state and v1 is never a goal state. */
	//The start state does not matter
    if (state == init) {
        /*
        assert(v1->includes(task_proxy.get_initial_state()));
        assert(!v2->includes(task_proxy.get_initial_state()));
        init = v1;
        */
        //TODO why ?
		
        assert(v1->includes(task_proxy.get_initial_state()) || v2->includes(task_proxy.get_initial_state()));
        if(v1->includes(task_proxy.get_initial_state())){
            init = v1;   
            //cout << "new Init 1" << endl;
        }
        else{
            init = v2;
            //cout << "new Init 1" << endl;
        }
		
        if (debug)
            cout << "New init state: " << *init << endl;
    }
    if (is_goal(state)) { 
	//if(is_abstract_goal(state)){
        
        goals.erase(state);
        goals.insert(v2);

        /*
        goals.erase(state);
        if(is_abstract_goal(v1)){
           goals.insert(v1);  
        }
        if(is_abstract_goal(v2)){
            goals.insert(v2);
        }
        */
        if (debug)
            cout << "New/additional goal state: " << *v2 << endl;
    }

    int num_states = get_num_states();
    if (num_states % 100000 == 0) {
        g_log << num_states << "/" << max_states << " states, "
              << transition_updater.get_num_non_loops() << "/"
              << max_non_looping_transitions << " transitions" << endl;
    }

    delete state;
    //cout << "RESULT STATE " << *v1 << endl;
    //cout << "RESULT STATE " << *v2 << endl;
    
    return make_pair(v1,v2);
}

unique_ptr<Flaw> Abstraction::find_flaw(const Solution &solution, AbstractState *start_state, State concrete_start_state){
	bool debug = false;
    if (debug)
        cout << "Find Flaw:" << endl;

    //AbstractState *abstract_state = init;
    //State concrete_state = task_proxy.get_initial_state();
    AbstractState *abstract_state = start_state;
    State concrete_state = concrete_start_state;
    
    assert(abstract_state->includes(concrete_state));

    if (debug){
        cout << "   Initial abstract state: " << *abstract_state << endl;
        cout << "   Length of solution: " << solution.size() << endl;
        for (const Transition &step : solution) {
            OperatorProxy op = task_proxy.get_operators()[step.op_id];
            cout << "   " << op.get_name() << endl;
        }
    }

    //int length_correct_trace = 0;
    for (const Transition &step : solution) {
        if (!utils::extra_memory_padding_is_reserved())
            break;
        OperatorProxy op = task_proxy.get_operators()[step.op_id];
        //cout << "operator " << op.get_name() << endl;       
        AbstractState *next_abstract_state = step.target;
        if (is_applicable(op, concrete_state)) {
            if (debug)
                cout << "  Move to " << *next_abstract_state << " with "
                     << op.get_name() << endl;
            State next_concrete_state = concrete_state.get_successor(op);
            if (!next_abstract_state->includes(next_concrete_state)) {
                if (debug)
                    cout << "  Paths deviate." << endl;
                ++deviations;
                //cout << "path deviate: " << length_correct_trace << "/" << solution.size() << endl;
				/*
                return utils::make_unique_ptr<Flaw>(
                    move(concrete_state),
                    abstract_state,
                    refinement_hierarchy.get_node(next_concrete_state)->get_AbstractState()->regress(op));
				*/
                return utils::make_unique_ptr<Flaw>(
                    move(concrete_state),
                    abstract_state,
                    next_abstract_state->regress(op));
            }
            abstract_state = next_abstract_state;
            concrete_state = move(next_concrete_state);
            //length_correct_trace++;
        } else {
            if (debug)
                cout << "  Operator not applicable: " << op.get_name() << endl;
            ++unmet_preconditions;
            //cout << "not app: " << length_correct_trace << "/" << solution.size() << endl;
            return utils::make_unique_ptr<Flaw>(
                move(concrete_state),
                abstract_state,
                AbstractState::get_abstract_state(
                    task_proxy, op.get_preconditions()));
        }
    }
    assert(is_goal(abstract_state));
    if (is_goal_state(task_proxy, concrete_state)) {
        // We found a concrete solution.
        return nullptr;
    } else {
        if (debug)
            cout << "  Goal test failed." << endl;
        ++unmet_goals;
        //cout << "goal: " << length_correct_trace << "/" << solution.size() << endl;
        return utils::make_unique_ptr<Flaw>(
            move(concrete_state),
            abstract_state,
            AbstractState::get_abstract_state(
                task_proxy, task_proxy.get_goals()));
    }
}

unique_ptr<Flaw> Abstraction::find_flaw_online(const Solution &solution, AbstractState *start_state, State concrete_start_state, vector<pair<int,vector<int>>> vars){
	bool debug = false;
    if (debug)
        cout << "Find Flaw:" << endl;

    //AbstractState *abstract_state = init;
    //State concrete_state = task_proxy.get_initial_state();
    AbstractState *abstract_state = start_state;
    State concrete_state = concrete_start_state;
    
    assert(abstract_state->includes(concrete_state));

    if (debug){
        cout << "   Initial abstract state: " << *abstract_state << endl;
        cout << "   Length of solution: " << solution.size() << endl;
        for (const Transition &step : solution) {
            OperatorProxy op = task_proxy.get_operators()[step.op_id];
            cout << "   " << op.get_name() << endl;
        }
    }

    //int length_correct_trace = 0;
    for (const Transition &step : solution) {
        if (!utils::extra_memory_padding_is_reserved())
            break;
        OperatorProxy op = task_proxy.get_operators()[step.op_id];
        //cout << "operator " << op.get_name() << endl;       
        AbstractState *next_abstract_state = step.target;
        if (is_applicable(op, concrete_state)) {
            if (debug)
                cout << "  Move to " << *next_abstract_state << " with "
                     << op.get_name() << endl;
            State next_concrete_state = concrete_state.get_successor(op);
            if (!next_abstract_state->includes(next_concrete_state)) {
                if (debug)
                    cout << "  Paths deviate." << endl;
                ++deviations;
                //cout << "path deviate: " << length_correct_trace << "/" << solution.size() << endl;
                return utils::make_unique_ptr<Flaw>(
                    move(concrete_state),
                    abstract_state,
                    next_abstract_state->regress(op));
            }
            abstract_state = next_abstract_state;
            concrete_state = move(next_concrete_state);
            //length_correct_trace++;
        } else {
            if (debug)
                cout << "   Operator not applicable: " << op.get_name() << endl;
            ++unmet_preconditions;
            //cout << "not app: " << length_correct_trace << "/" << solution.size() << endl;
            return utils::make_unique_ptr<Flaw>(
                move(concrete_state),
                abstract_state,
                AbstractState::get_abstract_state(
                    task_proxy, op.get_preconditions()));
        }
    }
    assert(is_goal(abstract_state) || is_refine_goal(abstract_state));
	bool is_new_goal = true;
	for(uint i = 0; i < vars.size(); i++){
		if(concrete_state[vars[i].first].get_value() != vars[i].second[0]){
			is_new_goal = false;
			break;
		}
	}
	//if (is_goal_state(task_proxy, concrete_state)) {
    if (is_new_goal) {
        // We found a concrete solution.
        return nullptr;
    } else {
        if (debug)
            cout << "  Goal test failed." << endl;
        ++unmet_goals;

		
        //cout << "goal: " << length_correct_trace << "/" << solution.size() << endl;
        return utils::make_unique_ptr<Flaw>(
            move(concrete_state),
            abstract_state,
            AbstractState::get_abstract_state_vector(
                task_proxy, vars));
    }
}
  
void Abstraction::update_h_and_g_values(int pos, bool new_order){
    
    update_timer.resume();
	//cout << "abstract_search.backwards_dijkstra" << endl;
    abstract_search.backwards_dijkstra(goals);
	
	
    if(new_order){
        for (AbstractState *state : states) {
            state->add_h_value(state->get_search_info().get_g_value());
        }
    }
    else{
		//cout << "update_h_and_g_values: " << pos << endl;
        for (AbstractState *state : states) {			
			//cout << *state << ": " << state->get_search_info().get_g_value() << endl;
            state->set_h_value(pos, state->get_search_info().get_g_value());
        }
    }
    
    // Update g values.
    // TODO: updating h values overwrites g values. Find better solution.
	//cout << "abstract_search.forward_dijkstra" << endl;
    abstract_search.forward_dijkstra(init);
    update_timer.stop();   
}
	
void Abstraction::update_h_values_complete_cost(){
	update_timer.resume();
    abstract_search.backwards_dijkstra(goals);
    for (AbstractState *state : states) {
        state->set_c_h(state->get_search_info().get_g_value());
    }
    // Update g values.
    // TODO: updating h values overwrites g values. Find better solution.
    abstract_search.forward_dijkstra(init);
    update_timer.stop();
}

void Abstraction::update_h_and_g_values() {
    update_timer.resume();
    abstract_search.backwards_dijkstra(goals);
    for (AbstractState *state : states) {
        state->set_h_value(state->get_search_info().get_g_value());
    }
    // Update g values.
    // TODO: updating h values overwrites g values. Find better solution.
    abstract_search.forward_dijkstra(init);
    update_timer.stop();
}

int Abstraction::get_h_value_of_initial_state() const {
    return init->get_h_value();
}

vector<int> Abstraction::get_saturated_costs(int order) {

    const int num_ops = task_proxy.get_operators().size();
    // Use value greater than -INF to avoid arithmetic difficulties.
    const int min_cost = use_general_costs ? -INF : 0;
    vector<int> saturated_costs(num_ops, min_cost);
    for (AbstractState *state : states) {
        const int g = state->get_search_info().get_g_value();
        const int h = state->get_h_value();

        /*
          No need to maintain goal distances of unreachable (g == INF)
          and dead end states (h == INF).

          Note that the "succ_h == INF" test below is sufficient for
          ignoring dead end states. The "h == INF" test is a speed
          optimization.
        */
        if (g == INF || h == INF)
            continue;

        for (const Transition &transition: state->get_outgoing_transitions()) {
            int op_id = transition.op_id;
            AbstractState *successor = transition.target;
            const int succ_h = successor->get_h_value();

            if (succ_h == INF)
                continue;

            int needed = h - succ_h;
            saturated_costs[op_id] = max(saturated_costs[op_id], needed);
        }

        if (use_general_costs) {
            /* To prevent negative cost cycles, all operators inducing
               self-loops must have non-negative costs. */
            for (int op_id : state->get_loops()) {
                saturated_costs[op_id] = max(saturated_costs[op_id], 0);
            }
        }
    }

    /*
    for(size_t i = 0; i < costs_partitioning.size(); i++){
        //cout << costs_partitioning[i] << " " << saturated_costs[i] << endl;
        if(costs_partitioning[i] != saturated_costs[i]){
            cout << "Cost of op " << i << " changed" << endl; 
        }
    }
    */
    if(order != -1){
        //cout << "Update saturated cost of order: " << order << " actions: " << saturated_costs.size() << endl;
        if((uint) order < costs_partitionings.size())
            costs_partitionings[order] = saturated_costs;
        else{
            costs_partitionings.push_back(saturated_costs);
        }
    }
    
	current_saturation.clear();
	for(size_t i = 0; i < saturated_costs.size(); i++){
			current_saturation.push_back(saturated_costs[i]);
	}
    
    return saturated_costs;
}
    
vector<int> Abstraction::get_costs_partitioning(int order){
    //cout << "Get costs partitionning of order: " << order << " of " << costs_partitionings.size() << " orders" << endl;
    assert((uint) order < costs_partitionings.size());
    return costs_partitionings[order];
}
    
void Abstraction::add_cost_partitioning(){
       costs_partitionings.push_back(current_saturation);
}

void Abstraction::print_statistics() {
    int total_incoming_transitions = 0;
    int total_outgoing_transitions = 0;
    int total_loops = 0;
    int dead_ends = 0;
    for (AbstractState *state : states) {
        if (state->get_h_value() == INF)
            ++dead_ends;
        total_incoming_transitions += state->get_incoming_transitions().size();
        total_outgoing_transitions += state->get_outgoing_transitions().size();
        total_loops += state->get_loops().size();
    }
    assert(total_outgoing_transitions == total_incoming_transitions);

    int total_cost = 0;
    for (OperatorProxy op : task_proxy.get_operators())
        total_cost += op.get_cost();

    cout << "Total operator cost: " << total_cost << endl;
    cout << "States: " << get_num_states() << endl;
    cout << "Dead ends: " << dead_ends << endl;
    cout << "Init h: " << get_h_value_of_initial_state() << endl;

    assert(transition_updater.get_num_loops() == total_loops);
    assert(transition_updater.get_num_non_loops() == total_outgoing_transitions);
    cout << "Looping transitions: " << total_loops << endl;
    cout << "Non-looping transitions: " << total_outgoing_transitions << endl;

    cout << "Deviations: " << deviations << endl;
    cout << "Unmet preconditions: " << unmet_preconditions << endl;
    cout << "Unmet goals: " << unmet_goals << endl;
    
    
    
    //cout << "Partitioning: " << endl << partition << endl;

}
    
void Abstraction::print_end_statistics() {
    cout << "Final States: " << get_num_states() << endl;
    cout << "Final Init h: " << get_h_value_of_initial_state() << endl;
    cout << "Final Deviations: " << deviations << endl;
    cout << "Final Unmet preconditions: " << unmet_preconditions << endl;
    cout << "Final Unmet goals: " << unmet_goals << endl;
    //cout << "update time: " << update_timer << endl;
    cout << "refinement time: " << refine_timer << endl;
	cout << "Usefull splits: " << usefull_splits << endl;
	cout << endl;
	cout << "Same abstract state: " << num_same_abstract_state << endl;
	cout << "Spurious meeting point: " << num_spurious_meeting_point << endl;
	cout << "Spurious goal meeting point: " << num_spurious_meeting_point_goal << endl;
	cout << "Standard refine: " << num_standard_refine << endl;
    
    /*
    for(vector<int> cp : costs_partitionings){
    for(int c : cp){
        cout << c << " ";   
    }
    cout << endl;
    }
    */
    cout << "-------------------------------------------------------" << endl;
}
    
void Abstraction::print_states(){
    cout << "+++++++++++++++++++ All States (" << states.size() << ") +++++++++++++++++++++" << endl;
	cout << "Init: " << *init << endl;
    for(AbstractState* state : states){            
        cout << "STATE G "<<  is_abstract_goal(state) << " h=" << state->get_h_values()[0] << " " << *state <<  endl;
		//cout << "STATE G "<<  is_abstract_goal(state) << " " << *state <<  endl;
        
		
		
        Transitions trans = state->get_outgoing_transitions();
        for(Transition t : trans){
			cout << "\t" << task_proxy.get_operators()[t.op_id].get_name() << endl;
            cout << "\t\t--> " << t.op_id << " " << *(t.target) << endl; 
		}
		
		/*
		cout << "	--> ";
		for(int i : state->get_loops()){
			cout << i << " ";		
		}
        cout << endl;
		*/
	
    }     
}
 
    
void Abstraction::print_cost(){
	for(vector<int> cp : costs_partitionings){
		for(int c : cp){
			cout << c << " ";   
		}
	cout << endl;
	}   
}
	
void Abstraction::print_cost(int order){
	for(int c : costs_partitionings[order]){
		cout << c << " ";   
	}
	cout << endl;
}
	
void Abstraction::print_current_cost(){
	int n = 0;
	for(int c : current_saturation){
		if(c > 0)
			cout << n << " "; 
		n++;
	}
	cout << endl;
}
}
