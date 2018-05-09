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
      current_eval_context(state_registry->get_initial_state(), &statistics),
      current_phase_start_g(-1),
	  time_unit(opts.get<double>("time_unit")),
	  lookahead_fraction(opts.get<double>("lookahead_fraction")),
	  use_refine_time_bound(opts.get<bool>("use_refine_time_bound")),
	  refine_base(opts.get<bool>("refine_base")),
	  refine_to_frontier(opts.get<bool>("refine_to_frontier")),
      num_ehc_phases(0),
      last_num_expanded(-1) {

	cout << "Heuristic: " << heuristic << endl;
    const GlobalState &initial_state = state_registry->get_initial_state();
    for (Heuristic *heuristic : heuristics) {
        heuristic->notify_initial_state(initial_state);
    }

	open_list = search_common::create_greedy_open_list_factory(opts)->create_state_open_list();
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

SearchStatus RealTimeSearch::compute_next_real_time_step(GlobalState s, bool solution_found, int){ // min_h){
	//find next action to execute
	Plan current_plan;
	search_space->trace_path(s, current_plan);

	//if goal was reached terminate the search and print the solution
	if(solution_found){
		//build corrent plan
		for(uint i = 0; i < current_plan.size(); i++){
			//cout << "NEXT ACTION ----> " << current_plan[i]->get_name() <<  " timestamp: " << utils::g_timer() << endl;
			game_time += time_unit;
			real_time_plan.push_back(current_plan[i]);
		}
		set_plan(real_time_plan);	
		return SOLVED;
	}

	//for(const GlobalOperator* next_action : current_plan){
		//cout << "Plan lenght: " << current_plan.size() << endl;
		const GlobalOperator* next_action = current_plan[0];
		real_time_plan.push_back(next_action);

		//cout << "NEXT ACTION ----> " << next_action->get_name() << " timestamp: " << utils::g_timer() << endl;
	
		//refine_root_to_frontier();
		double time_bound = use_refine_time_bound ? time_unit * (1-lookahead_fraction) : 1800;
		//cout << "Refine time bound: " << time_bound << endl;
		if(refine_to_frontier){
			refine_root_to_frontier(time_bound);
		}
		else{
			refine_expanded(time_bound);
		}

		//NEXT STATE
		GlobalState current_state = current_eval_context.get_state();
		SearchNode current_node = search_space->get_node(current_state);
		GlobalState next_state = state_registry->get_successor_state(current_state, *next_action);
		//cout << "Next state: " << next_state.get_id() << endl;
		int succ_g = current_node.get_g() + get_adjusted_cost(*next_action);
		EvaluationContext eval_context(next_state,  succ_g, true, &statistics);
		current_eval_context = eval_context;
		//only one step
	//	break;
	//}


	//Reset search 
	open_list->clear();
	delete search_space;
	search_space = new SearchSpace(*state_registry, cost_type);
	expand_states.clear();
	
	return IN_PROGRESS;
}

bool RealTimeSearch::refine_root_to_frontier(double time_bound){
	bool refined = false;
	while(! open_list->empty()){
		
        vector<int> last_key_removed;
        StateID id = open_list->remove_min(&last_key_removed);
        GlobalState new_goal = state_registry->lookup_state(id);
		vector<GlobalState> frontier_states;
		frontier_states.push_back(new_goal);


		GlobalState refine_state = current_eval_context.get_state();
		SearchNode node = search_space->get_node(refine_state);

		/*
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

		//TODO
		refined = heuristic->online_Refine(refine_state, succStates, frontier_states, time_bound) || refined;
	}
	return refined;
}

bool RealTimeSearch::refine_expanded(double time_bound){
	//cout << "---------------- Refine expanded ----------------------------------------------" << endl;
	bool refined = false;
	utils::Timer timer;
	timer.resume();
	utils::Timer iter_timer;
	timer.resume();

	//cout << "Expanded states: " << expand_states.size() << endl;

	list<StateID>::iterator it = expand_states.begin();
	while(timer() < time_bound){
		//cout << "Expanded: " << expand_states.size() << endl;
	    //cout << "Rest time: " << timer << " < " << time_bound << endl;
		StateID refine_state_id = *it;
		//expand_states.erase(expand_states.begin());
		//cout << "----------> STATE: " << refine_state_id << endl;
		//cout << "Refine state: " << refine_state_id << endl;
		GlobalState refine_state = state_registry->lookup_state(refine_state_id);
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
		if(applicable_ops.size() == 0){
			it++;
			continue;
		}
		vector<pair<GlobalState, int>> succStates;
		//cout << "Applicable ops: " << endl;
		for (const GlobalOperator *op : applicable_ops) {
			//cout << op->get_name() << endl;
			GlobalState succ_state = state_registry->get_successor_state(refine_state, *op);
			succStates.push_back(make_pair(succ_state, op->get_cost()));
		}

		refined = heuristic->online_Refine(refine_state, succStates, frontier_states, time_bound) || refined;
		time_bound -= iter_timer();
		iter_timer.reset();
		it++;
		if(it == expand_states.end()){
			if(refine_base){
				break;
			}
			it = expand_states.begin();
		}
	}

	//Refine base heuristic if there is still time
	if(refine_base && timer() < time_bound){
		//cout << "Refine base heuristic" << endl;
	    //cout << "Rest time: " << timer << " < " << time_bound << endl;
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

		refined = heuristic->online_Refine_base(refine_state, succStates, frontier_states, time_bound) || refined;

	}

	float diff = step_timer() > time_unit ? step_timer() - time_unit : 0;
	game_time = game_time + time_unit + diff;

	//cout << "Rest time: " << timer << " < " << time_bound << endl;
	return refined;
}

bool RealTimeSearch::refine_valley(GlobalState next_expanded_state, int min_h){

	//cout << "+++++++++++++ REFINE VALLY ++++++++++++++++" << endl;
	//cout << "MIN h: " << min_h << endl;
	//Collect minimal frontier nodes
	vector<GlobalState> frontier_min_states;
	frontier_min_states.push_back(next_expanded_state);

	//cout << "min frontier: " << next_expanded_state.get_id() << " h=" << min_h << endl; 
	while(!open_list->empty()){
		vector<int> keys;
		StateID id = open_list->remove_min(&keys);
		GlobalState s = state_registry->lookup_state(id);
		if(min_h == keys[0]){
			//cout << "min frontier: " << id << " h=" << keys[0] << endl; 
			frontier_min_states.push_back(s);
		}	
		else{
			break;
		}
	}

	//Trace back from F_0 to R to collect valles nodes
	//cout << "ROOT: "  << current_eval_context.get_state().get_id() << endl; 
	set<GlobalState> valley;
	valley.insert(current_eval_context.get_state());
	unordered_map<StateID, StateID> matching_refine_goal;
	matching_refine_goal.insert({current_eval_context.get_state().get_id(), frontier_min_states[0].get_id()});
	for(GlobalState f0 : frontier_min_states){
		GlobalState cs = f0;
		//valley.insert(cs);
		//matching_refine_goal.insert({f0.get_id(), f0.get_id()});
		//cout << "Start: " << cs.get_id() << " ";
		while(true){
			SearchNode node = search_space->get_node(cs);
			StateID parent_id = node.get_parent_id();
			//cout << "<-- " << parent_id << " ";
			GlobalState parent_state = state_registry->lookup_state(parent_id);
			if(parent_state.get_id() == current_eval_context.get_state().get_id()){
				break;
			}
			matching_refine_goal.insert({parent_id, f0.get_id()});
			//cout << parent_id << " --> " << f0.get_id() << endl;
			valley.insert(parent_state);
			cs=parent_state;
		}
		cout << endl;

	}
	/*
	cout << "Valley: ";
	for(GlobalState gs : valley){
		cout << gs.get_id() << " ";
	}
	cout << endl;
	*/

	//Refine all to low boarder nodes
	bool refined = false;
	for(GlobalState v_state : valley){
		//cout << "-------------- v state: " << v_state.get_id() << "------------" << endl;
		vector<const GlobalOperator *> applicable_ops;
		g_successor_generator->generate_applicable_ops(v_state, applicable_ops);
		vector<pair<GlobalState, int>> succStates;
		//cout << "Applicable ops: " << endl;
		for (const GlobalOperator *op : applicable_ops) {
			//cout << op->get_name() << endl;
			GlobalState succ_state = state_registry->get_successor_state(v_state, *op);
			//cout << "Valley state: " << v_state.get_id() << " boarder state: "  << succ_state.get_id() << endl;
			if(valley.find(succ_state) != valley.end()){
				//State is in valley
				//cout << " ---> state in valley " << endl;
				continue;
			}
			int h_v = heuristic->compute_heuristic(v_state);
			int h_s = heuristic->compute_heuristic(succ_state);
			if( h_v >= h_s){
				cout << h_v << " >= "  << h_s << " ?" << endl;
				//cout << "-------> refine: " << succ_state.get_id() << " "; 
				//h of boarder state is not high enought
				int bound = h_v + op->get_cost(); 
				//cout << "New goal state: " << matching_refine_goal.at(v_state.get_id()) << endl;
				GlobalState matching_goal =  state_registry->lookup_state(matching_refine_goal.at(v_state.get_id()));
				bool now_refined = heuristic->online_Refine(succ_state, matching_goal, bound);
				//cout << "----> refined: " << now_refined << endl;
				refined = refined || now_refined; 
			}
		}
	}
    //cout << "+++++++++++++++++++++++++++++++ END REFINE +++++++++++++++++++++++++++++++++++++" << endl;

	/*
	if(! refined){
		cout << "###################### REFINE NORMAL ################" << endl;
		GlobalState refine_state = current_eval_context.get_state();

		vector<const GlobalOperator *> applicable_ops;
		g_successor_generator->generate_applicable_ops(refine_state, applicable_ops);
		vector<pair<GlobalState, int>> succStates;
		for (const GlobalOperator *op : applicable_ops) {
			GlobalState succ_state = state_registry->get_successor_state(refine_state, *op);
			succStates.push_back(make_pair(succ_state, op->get_cost()));
		}
	
	
		vector<GlobalState> frontier_states;
		vector<pair<int,int>> pre_con;

		heuristic->online_Refine(refine_state, succStates, frontier_states, pre_con);
	}
	*/

	return refined;
}

SearchStatus RealTimeSearch::step() {
	//cout << "+++++++++++++ STEP +++++++++++++++" << endl;
	num_ehc_phases++;
    last_num_expanded = statistics.get_expanded();
    search_progress.check_progress(current_eval_context);

	/*
    if (check_goal_and_set_plan(current_eval_context.get_state())) {
        return SOLVED;
    }
	*/

	//Add initial state of current search to the openlist
	GlobalState current_state = current_eval_context.get_state();
    statistics.inc_evaluated_states();
	SearchNode node = search_space->get_node(current_state);
	node.open_initial();
	open_list->insert(current_eval_context, current_state.get_id());

	step_timer.reset();
    return  search();
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
		node.close();
		//cout << "Expand: " << state.get_id() << " h=" << last_key_removed[0] << endl;
		//lookahead--;
		statistics.inc_expanded(1);
		expand_states.push_front(state.get_id());

		//If solution has been found or lookhead is reached return the current
		//best state (next min in openlist)
		bool solution_found = check_goal_and_set_plan(state);
		if((step_timer() >= (time_unit * lookahead_fraction) || solution_found)){
			expand_states.push_back(state.get_id());
			//cout << "----> compute next real time step" << endl;
			return compute_next_real_time_step(state, solution_found, last_key_removed[0]);
		}
		
		vector<const GlobalOperator *> applicable_ops;
    	g_successor_generator->generate_applicable_ops(state, applicable_ops);

		//bool new_state_found = false;
		for (const GlobalOperator *op : applicable_ops) {

			statistics.inc_generated();
			
			GlobalState succ_state = state_registry->get_successor_state(state, *op);
			SearchNode succ_node = search_space->get_node(succ_state);

			// Previously encountered dead end. Don't re-evaluate.
			if (succ_node.is_dead_end())
				continue;

			// update new path
			if (succ_node.is_new()) {
				//cout << "Succ: " << succ_state.get_id() << "   " << op->get_name();
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
    parser.add_option<double>("time_unit","TODO", "1");
    parser.add_option<double>("lookahead_fraction","TODO", "0.1");
    parser.add_option<bool>("use_refine_time_bound","TODO", "true");
    parser.add_option<bool>("refine_base","TODO", "false");
    parser.add_option<bool>("refine_to_frontier","TODO", "false");



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
