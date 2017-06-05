#include "abstraction.h"

#include "abstract_state.h"
#include "utils.h"

#include "../heuristic.h"
#include "../globals.h"
#include "../task_tools.h"

#include "../utils/logging.h"
#include "../utils/memory.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <unordered_map>

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
                assert(!wanted.empty());
                splits.emplace_back(var_id, move(wanted));
            }
        }
        assert(!splits.empty());
        return splits;
    }
};

    
Abstraction::Abstraction(
    const shared_ptr<AbstractTask> task,
    int max_states,
    int max_non_looping_transitions,
    double max_time,
    bool use_general_costs,
    PickSplit pick,
    utils::RandomNumberGenerator &rng,
    bool debug)
    : task_proxy(*task),
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
      debug(debug),
      rng(rng){
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
          
    print_statistics();
}


Abstraction::~Abstraction() {
    cout << "Abstruction deleted" << endl;
    for (AbstractState *state : states)
        delete state;
}

bool Abstraction::is_goal(AbstractState *state) const {
    return goals.count(state) == 1;
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
        separate_facts_unreachable_before_goal();
    }
    bool found_concrete_solution = false;
    while (may_keep_refining()) {
        bool found_abstract_solution = abstract_search.find_solution(init, goals);
        if (!found_abstract_solution) {
            cout << "Abstract problem is unsolvable!" << endl;
            break;
        }
        unique_ptr<Flaw> flaw = find_flaw(abstract_search.get_solution(), init, task_proxy.get_initial_state());
        if (!flaw) {
            found_concrete_solution = true;
            break;
        }
        AbstractState *abstract_state = flaw->current_abstract_state;
        vector<Split> splits = flaw->get_possible_splits();
        const Split &split = split_selector.pick_split(*abstract_state, splits, rng);
        refine(abstract_state, split.var_id, split.values);
    }
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
    
int Abstraction::onlineRefine(const State &state, int num_of_Iter, int max_states_refine){
    
    int state_border = get_num_states() + max_states_refine < 0 ? max_states_refine : get_num_states() + max_states_refine;
    if(!(utils::extra_memory_padding_is_reserved() && get_num_states() < state_border && num_of_Iter >= 0)){
     //cout << "Do not refine state_border: " << state_border << endl;
     return 0;   
    }
    refinement_calls++;
    int refined_states = 0;
    int old_h = get_node(state)->get_h_value();
    int current_h_value = old_h;
    int iter_until_recompute = 0;
    refine_timer.resume();
	while((utils::extra_memory_padding_is_reserved() && get_num_states() < state_border && num_of_Iter >= 0) &&
          current_h_value <= old_h){
        
        
        AbstractState *start_state = get_node(state)->get_AbstractState(); 
        bool found_abstract_solution = abstract_search.find_solution(start_state, goals);

        if (!found_abstract_solution) {
            cout << "Abstract problem is unsolvable!" << endl;
            break;
        }
        
        //Find flaw starting from the spesified state "start_state"
        unique_ptr<Flaw> flaw = find_flaw(abstract_search.get_solution(), start_state, state);
        if (!flaw) {
            //TODO solution found
            break;
        }
        AbstractState *abstract_state = flaw->current_abstract_state;
        vector<Split> splits = flaw->get_possible_splits();
        const Split &split = split_selector.pick_split(*abstract_state, splits, rng);
        refine(abstract_state, split.var_id, split.values);  
        
        refined_states++;
		num_of_Iter--;
        iter_until_recompute++;
        //recompute h and g values and check if the heuristic value increased
        if(iter_until_recompute == 10){
            iter_until_recompute = 0;
            break;
            //update_h_and_g_values();
            //current_h_value = get_node(state)->get_h_value();
            if(debug){
                if(current_h_value > old_h){
                    cout << "h increased after " << (num_of_Iter) << " iterations from " << old_h << " to " << current_h_value << endl;   
                }
            }
        }      
   }
    refine_timer.stop();
    //cout << "refinement calls: " <<  refinement_calls << endl;
    if(refinement_calls % 1 == 0){
        //cout << "Update h and g values" << endl;
        update_timer.resume();
        update_h_and_g_values();
        update_timer.stop();
        //cout << "update h values: " << update_timer << endl;
        //current_h_value = get_node(state)->get_h_value();
        //cout << "h increased from " << old_h << " to " << current_h_value << endl; 
    }
   return refined_states;
}

void Abstraction::refine(AbstractState *state, int var, const vector<int> &wanted) {
    if (debug)
        cout << "Refine " << *state << " for " << var << "=" << wanted << endl;
    pair<AbstractState *, AbstractState *> new_states = state->split(var, wanted);
    AbstractState *v1 = new_states.first;
    AbstractState *v2 = new_states.second;

    transition_updater.rewire(state, v1, v2, var);

    states.erase(state);
    states.insert(v1);
    states.insert(v2);

    /* Since the search is always started from the abstract initial state, v2
       is never the new initial state and v1 is never a goal state. */
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
        }
        else{
            init = v2;
        }
          
        if (debug)
            cout << "New init state: " << *init << endl;
    }
    if (is_goal(state)) {
        /*
        goals.erase(state);
        goals.insert(v2);
        */
        //TODO why ?
        
        goals.erase(state);
        if(is_goal(v1)){
           goals.insert(v1);  
        }
        else{
            goals.insert(v2);
        }
        
        if (debug)
            cout << "New/additional goal state: " << *v2 << endl;
    }

    int num_states = get_num_states();
    if (num_states % 1000 == 0) {
        g_log << num_states << "/" << max_states << " states, "
              << transition_updater.get_num_non_loops() << "/"
              << max_non_looping_transitions << " transitions"        
                << " update: " << update_timer
                << " refine: " << refine_timer << endl;
    }

    delete state;
}

// TODO find flow start at specified state
unique_ptr<Flaw> Abstraction::find_flaw(const Solution &solution, AbstractState *start_state, State concrete_start_state){
    if (debug)
        cout << "Check solution:" << endl;

    //AbstractState *abstract_state = init;
    //State concrete_state = task_proxy.get_initial_state();
    AbstractState *abstract_state = start_state;
    State concrete_state = concrete_start_state;
    
    assert(abstract_state->includes(concrete_state));

    if (debug){
        cout << "  Initial abstract state: " << *abstract_state << endl;
        cout << "  Length of solution: " << solution.size() << endl;
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

void Abstraction::update_h_and_g_values() {
    abstract_search.backwards_dijkstra(goals);
    for (AbstractState *state : states) {
        state->set_h_value(state->get_search_info().get_g_value());
    }
    // Update g values.
    // TODO: updating h values overwrites g values. Find better solution.
    abstract_search.forward_dijkstra(init);
}

int Abstraction::get_h_value_of_initial_state() const {
    return init->get_h_value();
}

vector<int> Abstraction::get_saturated_costs() {
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
    return saturated_costs;
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
    cout << endl;
    cout << "Timer update h values: " << update_timer << endl;
    cout << "Timer refine states: " << refine_timer << endl;
}
}
