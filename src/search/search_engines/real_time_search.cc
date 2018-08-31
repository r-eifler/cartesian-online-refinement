#include "real_time_search.h"

#include "search_common.h"

#include "../global_operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../successor_generator.h"

#include "../evaluators/g_evaluator.h"
#include "../evaluators/pref_evaluator.h"

#include "../open_lists/open_list_factory.h"
#include "../open_lists/standard_scalar_open_list.h"
#include "../open_lists/tiebreaking_open_list.h"

#include "../algorithms/ordered_set.h"

#include "../utils/system.h"

#include "limits.h"

using namespace std;
using utils::ExitCode;

namespace real_time_search {
using GEval = g_evaluator::GEvaluator;
using PrefEval = pref_evaluator::PrefEvaluator;


RealTimeSearch::RealTimeSearch(
    const Options &opts)
    : SearchEngine(opts),
      heuristic(opts.get<Heuristic *>("h")),
      preferred_operator_heuristics(opts.get_list<Heuristic *>("preferred")),
      preferred_usage(PreferredUsage(opts.get_enum("preferred_usage"))),
      learn_strategy(LearnStrategy(opts.get_enum("learn_strategy"))),
      current_eval_context(state_registry->get_initial_state(), 0, true, &statistics),
      current_phase_start_g(-1),
	  time_unit(opts.get<double>("time_unit")),
	  lookahead_fraction(opts.get<double>("lookahead_fraction")),
	  use_refine_time_bound(opts.get<bool>("use_refine_time_bound")),
	  refine_base(opts.get<bool>("refine_base")),
	  refine_to_frontier(opts.get<bool>("refine_to_frontier")),
      num_ehc_phases(0),
      last_num_expanded(-1) {

	//cout << "Heuristic: " << heuristic << endl;
    const GlobalState &initial_state = state_registry->get_initial_state();
    for (Heuristic *heuristic : heuristics) {
        heuristic->notify_initial_state(initial_state);
    }

	open_list = search_common::create_astar_open_list_factory_and_f_eval(opts).first->create_state_open_list();
	open_list_learn = search_common::create_greedy_open_list_factory(opts)->create_state_open_list();
}

RealTimeSearch::~RealTimeSearch() {
}

void RealTimeSearch::reach_state(
    const GlobalState &parent, const GlobalOperator &op, const GlobalState &state) {
    for (Heuristic *heur : heuristics) {
        heur->notify_state_transition(parent, op, state);
    }
}

void RealTimeSearch::initialize() {
    assert(heuristic);
    cout << "Conducting real-time search" << endl;

    bool dead_end = current_eval_context.is_heuristic_infinite(heuristic);
    statistics.inc_evaluated_states();
    print_initial_h_values(current_eval_context);

    if (dead_end) {
        cout << "Initial state is a dead end, no solution" << endl;
        if (heuristic->dead_ends_are_reliable())
            utils::exit_with(ExitCode::UNSOLVABLE);
        else
            utils::exit_with(ExitCode::UNSOLVED_INCOMPLETE);
    }

    //SearchNode node = search_space->get_node(current_eval_context.get_state());
    //node.open_initial();

    current_phase_start_g = 0;
}

bool RealTimeSearch::compute_next_real_time_step(const GlobalState &s, bool solution_found){
	//cout << "Next state to expand: " << s.get_id() << endl;
	//if goal was reached terminate the search and print the solution
	if(solution_found){
		Plan current_plan;
		search_space->trace_path(s, current_plan);
		//build corrent plan
		for(uint i = 0; i < current_plan.size(); i++){
			//cout << "NEXT ACTION ----> " << current_plan[i]->get_name() <<  " timestamp: " << utils::g_timer() << endl;
			game_time += time_unit;
			real_time_plan.push_back(current_plan[i]);
		}
		set_plan(real_time_plan);	
		return true;
	}

	SearchNode node = search_space->get_node(s);
	const GlobalOperator* next_action = node.get_best_next_action(); 
	real_time_plan.push_back(next_action);

	//cout << "NEXT ACTION ----> " << next_action->get_name() << " timestamp: " << utils::g_timer() << endl;

	return false;
}


bool RealTimeSearch::update_heuristic(const GlobalState &s){

	double time_bound = use_refine_time_bound ? time_unit * (1-lookahead_fraction) : 1800;
	//TODO
	//add other refinement methods
	return bellman_dijkstra_backup(time_bound, s);
}


void RealTimeSearch::reset_search_and_execute_next_step(const GlobalState &s){
	SearchNode node = search_space->get_node(s);
	const GlobalOperator* next_action = node.get_best_next_action(); 
	//cout << "StateID: " << s.get_id() << endl;
	//cout << "NEXT ACTION ----> " << next_action->get_name() << endl;

	GlobalState current_state = current_eval_context.get_state();
	SearchNode current_node = search_space->get_node(current_state);
	GlobalState next_state = state_registry->get_successor_state(current_state, *next_action);
	//cout << "Next state: " << next_state.get_id() << endl;
	int succ_g = current_node.get_g() + get_adjusted_cost(*next_action);
	EvaluationContext eval_context(next_state,  succ_g, true, &statistics);
	current_eval_context = eval_context;


	//reset search
	open_list->clear();
	delete search_space;
	search_space = new SearchSpace(*state_registry, cost_type);
	expand_states.clear();
}

/*
 *	Refine the abstraction based on the root node of the current lookahead.
 *	As the goal of the refinement step the frontier nodes are used.
 *	The idea behind the approach is a more local refinement
 */
bool RealTimeSearch::refine_root_to_frontier(double time_bound){
	bool refined = false;
	while(! open_list->empty()){
		
        vector<int> last_key_removed;
        StateID id = open_list->remove_min(&last_key_removed);
        GlobalState new_goal = state_registry->lookup_state(id);
		//frontier state which is the goals in the refinement step
		vector<GlobalState> goal_states;
		goal_states.push_back(new_goal);

		//The current root node is refined
		GlobalState refine_state = current_eval_context.get_state();
		SearchNode node = search_space->get_node(refine_state);

		/*
		cout << ".......Refine State........" << endl;
		refine_state.dump_pddl();
		cout << endl;
		cout << "++++++++++++++++++" << endl;
		*/

		//Compute successor states
		vector<const GlobalOperator *> applicable_ops;
		g_successor_generator->generate_applicable_ops(refine_state, applicable_ops);
		vector<pair<GlobalState, int>> succStates;
		for (const GlobalOperator *op : applicable_ops) {
			GlobalState succ_state = state_registry->get_successor_state(refine_state, *op);
			succStates.push_back(make_pair(succ_state, op->get_cost()));
		}

		refined = heuristic->online_Refine(refine_state, succStates, goal_states, time_bound) || refined;
	}
	return refined;
}

bool RealTimeSearch::refine_heuristic(double time_bound){
	GlobalState refine_state = current_eval_context.get_state();
	SearchNode node = search_space->get_node(refine_state);

	vector<GlobalState> frontier_states;
	/*
	cout << "...... Parent State......." << endl;
	parent_state.dump_pddl();
	cout << endl;
	cout << ".......Refine State........" << endl;
	refine_state.dump_pddl();
	cout << endl;
	cout << "++++++++++++++++++" << endl;
	*/

	//GlobalState refine_state = next_state;
	vector<const GlobalOperator *> applicable_ops;
	g_successor_generator->generate_applicable_ops(refine_state, applicable_ops);
	vector<pair<GlobalState, int>> succStates;
	//cout << "Applicable ops: " << endl;
	for (const GlobalOperator *op : applicable_ops) {
		//cout << op->get_name() << endl;
		GlobalState succ_state = state_registry->get_successor_state(refine_state, *op);
		succStates.push_back(make_pair(succ_state, op->get_cost()));
	}

	bool refined = false;
	if(learn_strategy == LearnStrategy::BELLMAN_AND_REFINE){
		refined = heuristic->online_Refine_base(refine_state, succStates, frontier_states, time_bound);
	}
	if(learn_strategy == LearnStrategy::REFINE){
		refined = heuristic->online_Refine(refine_state, succStates, frontier_states, time_bound);
	}


	//cout << "Rest time: " << timer << " < " << time_bound << endl;
	return refined;
}

/*
 *	Refine the heuristic on all expanded states, beginning in the current root node
 */
//bool RealTimeSearch::bellman_dijkstra_backup(double time_bound, const GlobalState &s){
bool RealTimeSearch::bellman_dijkstra_backup(double, const GlobalState &s){
	//cout << "---------------- Refine expanded ----------------------------------------------" << endl;
	utils::Timer timer;
	timer.resume();

	//update h of all closed states to infinity
	list<StateID>::iterator it = expand_states.begin();
	while(it != expand_states.end()){
		StateID refine_state_id = *it;
		GlobalState refine_state = state_registry->lookup_state(refine_state_id);
		//assert(refine_state.is_closed());
		heuristic->update(refine_state, INT_MAX);
		//cout << "INFTY state: " << refine_state.get_id() << endl;
		it++;
	}

	//Initialize the openlist of dijkstra with the openlist of Astar
	//cout << "Dijkstra openlist: " << endl;
	//ad last expanded state
	open_list_learn->clear();
	int openlist_size = 0;
	EvaluationContext eval_context(s, 0, true, &statistics);
	open_list_learn->insert(eval_context, s.get_id());
	//cout << s.get_id() << endl;
	while(! open_list->empty()){
		vector<int> last_key_removed;
		StateID state_id = open_list->remove_min(&last_key_removed);
		GlobalState state = state_registry->lookup_state(state_id);
		//cout << state_id << endl;

		EvaluationContext eval_context(state, 0, true, &statistics);
		open_list_learn->insert(eval_context, state_id);
		openlist_size++;
	}

	while(!open_list_learn->empty()){
	//while(timer() < time_bound && !open_list_learn->empty()){
		//cout << timer() << " < " << time_bound << endl;
		//cout << "----------------------" << openlist_size << "----------------------------------" << endl;

		vector<int> last_key_removed;
		StateID refine_state_id = open_list_learn->remove_min(&last_key_removed);
		openlist_size--;
		//cout << "Child: " << refine_state_id << endl;
		GlobalState refine_state = state_registry->lookup_state(refine_state_id);
		SearchNode node = search_space->get_node(refine_state);

		//heuristic of the child
		EvaluationContext eval_context(refine_state, 0, true, &statistics);
		int h_child = eval_context.get_heuristic_value_or_infinity(heuristic);

		vector<pair<StateID, const GlobalOperator *>> parent_state_ids = node.get_parent_ids();

		for(uint i = 0; i < parent_state_ids.size(); i++){

			StateID parent_id = parent_state_ids[i].first;
			//cout << "parent: " << parent_id << endl;
			GlobalState parent_state = state_registry->lookup_state(parent_id);
			SearchNode parent_node = search_space->get_node(parent_state);

			if (! parent_node.is_closed()){
				continue;
			}
			EvaluationContext eval_context_parent(parent_state, 0, true, &statistics);
			int h_parent = eval_context_parent.get_heuristic_value_or_infinity(heuristic);
			
			//check bellman
			//cout << h_parent << " > " << h_child << " + " << parent_state_ids[i].second->get_cost() << endl;
			if(h_parent > h_child + parent_state_ids[i].second->get_cost()){
				heuristic->update(parent_state, h_child +  parent_state_ids[i].second->get_cost());
				//cout << "\t --> update" << endl;

				//insert parent-state into open
				EvaluationContext eval_context_openlist(parent_state, 0, true, &statistics);
				open_list_learn->insert(eval_context_openlist, parent_id);
				openlist_size++;
				
			}
		}
	}

	return true;
}

SearchStatus RealTimeSearch::step() {
	//cout << "+++++++++++++++++++++++++++++++++++++++++++++++++ STEP ++++++++++++++++++++++++++++++++++++++++" << endl;
	//cout << "Curretn root state: "  << current_eval_context.get_state().get_id() << endl;
	//
	if(print_timer() > 60){
		print_statistics();
		print_timer.reset();
	}
	
	//Time step begins	
	step_timer.reset();


	num_ehc_phases++;
    last_num_expanded = statistics.get_expanded();
    search_progress.check_progress(current_eval_context);

	//Phase 1
	//-----------------------------------------------------------------------------------------------------
	//cout << "*********** Phase 1: UPDATE ***************" << endl;
	//First update the heuristic based on the previous lookahead
	//double update_time = 0;
	//bool updated_heuristic = false;
	if(expand_states.size() > 0 && (learn_strategy == LearnStrategy::BELLMAN || learn_strategy == LearnStrategy::BELLMAN_AND_REFINE)){
		//cout << "-------------------- bellman updates ------------------- " << endl;
		//updated_heuristic = true;
		update_heuristic(next_expanded_state[0]);
		if(learn_strategy == LearnStrategy::BELLMAN){
			reset_search_and_execute_next_step(next_expanded_state[0]);
		}
		//update_time = step_timer();
		//cout << "Fraction used for update: " << (update_time / time_unit) << endl; 
	}

	if(expand_states.size() > 0 && learn_strategy == LearnStrategy::BELLMAN_AND_REFINE){
		if(step_timer() < time_unit){
			//cout << "-------------------- Refine heuristic ------------------- " << endl;
			double refine_time = (time_unit - step_timer()) * (1-lookahead_fraction); 
			//cout << "Refine time: " << refine_time << endl;
			refine_heuristic(refine_time);
		}
		reset_search_and_execute_next_step(next_expanded_state[0]);
	}




	//TODO
	// you have to expand at least one state otherwise you can not commit to an action

	//Phase 2
	//-----------------------------------------------------------------------------------------------------
	//lookahead search
	//cout << "*********** Phase 2: SEARCH  ***************" << endl;

	//Add initial state of current search to the openlist
	GlobalState current_state = current_eval_context.get_state();
	//cout << "Current State: " << current_state.get_id() << endl;
    statistics.inc_evaluated_states();
	SearchNode node = search_space->get_node(current_state);
	node.open_initial();
	open_list->insert(current_eval_context, current_state.get_id());


	lookahead_time = time_unit;
	//if(!updated_heuristic){
	//	lookahead_time -= lookahead_fraction * time_unit;
	//}
	if(learn_strategy == LearnStrategy::REFINE){
		lookahead_time -= lookahead_fraction * time_unit;
	}
	//cout << "Lookahead time deadline: " << lookahead_time << endl;
	SearchStatus status = search();
	if(status != SearchStatus::IN_PROGRESS){
		return status;
	}
	//cout << "Fraction used for lookahead: " << ((step_timer() - update_time) / time_unit) << endl; 
	//-----------------------------------------------------------------------------------------------------

	//For the only refine strategy refine after search
	if(learn_strategy == LearnStrategy::REFINE){
		//cout << "Only refine after search" << endl;
		if(step_timer() < time_unit ){
			//cout << "-------------------- Refine heuristic ------------------- " << endl;
			refine_heuristic(time_unit - step_timer());
		}
		reset_search_and_execute_next_step(next_expanded_state[0]);
	}
	//cout << "Step complete: " << step_timer() << endl;


	float diff = step_timer() > time_unit ? step_timer() - time_unit : 0;
	game_time = game_time + time_unit + diff;

    return  status;
}

SearchStatus RealTimeSearch::search() {
	//int lookahead = 5;

    while (!open_list->empty()) {
		//cout << "--------- Lookahead: " << lookahead << "----------" << endl;
        vector<int> last_key_removed;
        StateID id = open_list->remove_min(&last_key_removed);

        GlobalState state = state_registry->lookup_state(id);
        SearchNode node = search_space->get_node(state);

		//A* check closed 
		if(node.is_closed()){
			continue;
		}


		//If solution has been found or lookhead is reached return the current
		//best state (next min in openlist)
		bool solution_found = check_goal_and_set_plan(state);
		//cout << step_timer() << " >= " << (time_unit * lookahead_fraction) << endl;
		//if((step_timer() >= (time_unit * lookahead_fraction) || solution_found)){
		//TODO how to ensure that at least one state is expanded ?
		if((step_timer() >= (lookahead_time - 0.0005) && expand_states.size() > 0 ) || solution_found){
			//cout << "Best frontier node (expanded next): " << state.get_id() << endl;
			//expand_states.push_back(state.get_id());
			assert(expand_states.size() > 0);
			//cout << "Expansions: " << expand_states.size() << endl;
			next_expanded_state.clear();
			next_expanded_state.push_back(state);
			bool solved = compute_next_real_time_step(state, solution_found);
			return solved ? SearchStatus::SOLVED : SearchStatus::IN_PROGRESS; 
		}

		node.close();
		//cout << "Expand: " << state.get_id() << " h=" << last_key_removed[0] << endl;
		//lookahead--;
		statistics.inc_expanded(1);
		expand_states.push_front(state.get_id());
		
		vector<const GlobalOperator *> applicable_ops;
    	g_successor_generator->generate_applicable_ops(state, applicable_ops);

		//bool new_state_found = false;
		for (const GlobalOperator *op : applicable_ops) {
			//cout << "---------------------------------------------------" << endl;
			statistics.inc_generated();
			
			GlobalState succ_state = state_registry->get_successor_state(state, *op);
			SearchNode succ_node = search_space->get_node(succ_state);
		
			//store parent for reverse dijkstra in learning phase
			succ_node.add_parent(node, op);
			//cout << state.get_id() << " -> "  << succ_state.get_id() << endl;
			

			// Previously encountered dead end. Don't re-evaluate.
			if (succ_node.is_dead_end())
				continue;

			// update new path
			if (succ_node.is_new()) {

				heuristic->check_heuristic_improved(succ_state);


				//cout << "Succ: " << succ_state.get_id() << "   " << op->get_cost() << " "  << op->get_name() << endl;
				//new_state_found = true;
				//cout << "Expand: " << op->get_name() << endl;
					/*
				  Note: we must call notify_state_transition for each heuristic, so
				  don't break out of the for loop early.
				*/
				for (Heuristic *heuristic : heuristics) {
					heuristic->notify_state_transition(state, *op, succ_state);
				}

				// We have not seen this state before.
				// Evaluate and create a new node.

				// Careful: succ_node.get_g() is not available here yet,
				// hence the stupid computation of succ_g.
				// TODO: Make this less fragile.
				int succ_g = node.get_g() + get_adjusted_cost(*op);

				EvaluationContext eval_context(
					succ_state, succ_g, false, &statistics);
				statistics.inc_evaluated_states();

				if (open_list->is_dead_end(eval_context)) {
					succ_node.mark_as_dead_end();
					statistics.inc_dead_ends();
					continue;
				}


				succ_node.open(node, op);
				//child gets the best next state from its parent
				const GlobalOperator* best_next_action = node.get_best_next_action();
				if (best_next_action == NULL){
					//if the best_next_action of the parent is not set then the action to the state itself is the best next action
					succ_node.set_best_next_action(op);
				}
				else{
					succ_node.set_best_next_action(best_next_action);
				}


				//succ_state.dump_pddl();
				//cout << " h=" << eval_context.get_heuristic_value(heuristic) << endl;
				//cout << "Update Parent new " << succ_state.get_id() << " -> " << state.get_id() << endl;
			
				open_list->insert(eval_context, succ_state.get_id());

				if (search_progress.check_progress(eval_context)) {
					//TODO
				}
			} 
			else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
				//cout << "UPDATE Parent: Succ: " << succ_state.get_id() << "   " << op->get_name();
				// If we do not reopen closed nodes, we just update the parent pointers.
				// Note that this could cause an incompatibility between
				// the g-value and the actual path that is traced back.
				//cout << "Update Parent no reopen " << succ_state.get_id() << " -> " << state.get_id() << endl;
				succ_node.update_parent(node, op);
				//also update best next state if the best parent state changes
				const GlobalOperator* best_next_action = node.get_best_next_action();
				if (best_next_action == NULL){
					//if the best_next_action of the parent is not set then the action to the state itself is the best next action
					succ_node.set_best_next_action(op);
				}
				else{
					succ_node.set_best_next_action(best_next_action);
				}
			}
			/*
			else
			{
				cout << "NOT NEW: Succ: " << succ_state.get_id() << "   " << op->get_name() << endl;
			}
			*/
        }

		/*
		if(! new_state_found){
			cout << "~~~~~~~~~~~~~~~~~~~~~ Refine Deadend ~~~~~~~~~~~~~~~ " << endl;
			heuristic->online_Refine(state, current_eval_context.get_state(),current_eval_context.get_heuristic_value(heuristic));
		}
		*/
    }
    cout << "No solution - FAILED" << endl;
    return FAILED;
}

void RealTimeSearch::print_statistics() const {
    statistics.print_detailed_statistics();

   cout << "Game-time: " << game_time << endl; 
   cout << "Number of steps: " << num_ehc_phases << endl;

	cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
	heuristic->print_statistics();
}

static SearchEngine *_parse(OptionParser &parser) {
    parser.document_synopsis("Lazy enforced hill-climbing", "");
    parser.add_option<Heuristic *>("h", "heuristic");
    vector<string> preferred_usages;
    preferred_usages.push_back("PRUNE_BY_PREFERRED");
    preferred_usages.push_back("RANK_PREFERRED_FIRST");
    parser.add_enum_option(
        "preferred_usage",
        preferred_usages,
        "preferred operator usage",
        "PRUNE_BY_PREFERRED");
    parser.add_list_option<Heuristic *>(
        "preferred",
        "use preferred operators of these heuristics",
        "[]");
    parser.add_option<int>(
        "boost",
        "boost value for preferred operator open lists", "0");
    parser.add_list_option<ScalarEvaluator *>("evals", "scalar evaluators");
	parser.add_option<ScalarEvaluator *>("eval", "scalar evaluator");
    parser.add_option<double>("time_unit","TODO", "1");
    parser.add_option<double>("lookahead_fraction","TODO", "0.1");
    parser.add_option<bool>("use_refine_time_bound","TODO", "true");
    parser.add_option<bool>("refine_base","TODO", "false");
    parser.add_option<bool>("refine_to_frontier","TODO", "false");

	vector<string> learn_strategies;
    learn_strategies.push_back("BELLMAN");
    learn_strategies.push_back("BELLMAN_AND_REFINE");
    learn_strategies.push_back("REFINE");
    parser.add_enum_option(
        "learn_strategy", learn_strategies, "TODO", "BELLMAN");


    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();
    opts.verify_list_non_empty<ScalarEvaluator *>("evals");

    if (parser.dry_run())
        return nullptr;
    else
        return new RealTimeSearch(opts);
}

static Plugin<SearchEngine> _plugin("rt", _parse);
}
