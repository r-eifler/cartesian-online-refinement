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
    cout << "Conducting enforced hill-climbing search, (real) bound = "
         << bound << endl;
    if (use_preferred) {
        cout << "Using preferred operators for "
             << (preferred_usage == PreferredUsage::RANK_PREFERRED_FIRST ?
            "ranking successors" : "pruning") << endl;
    }

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

SearchStatus RealTimeSearch::compute_next_real_time_step(GlobalState s, bool solution_found){
	//find next action to execute
	Plan current_plan;
	search_space->trace_path(s, current_plan);

	//if goal was reached terminate the search and print the solution
	if(solution_found){
		//build corrent plan
		for(uint i = 0; i < current_plan.size(); i++){
			cout << "NEXT ACTION ----> " << current_plan[i]->get_name() << endl;
			real_time_plan.push_back(current_plan[i]);
		}
		set_plan(real_time_plan);	
		return SOLVED;
	}

	for(const GlobalOperator* next_action : current_plan){
		//const GlobalOperator* next_action = current_plan[0];
		real_time_plan.push_back(next_action);

		//cout << "Length of plan part: " << current_plan.size() << " Length of path: " << real_time_plan.size() << endl;

		cout << "NEXT ACTION ----> " << next_action->get_name() << endl;
			
		//NEXT STATE
		GlobalState current_state = current_eval_context.get_state();
		SearchNode current_node = search_space->get_node(current_state);
		GlobalState next_state = state_registry->get_successor_state(current_state, *next_action);
		int succ_g = current_node.get_g() + get_adjusted_cost(*next_action);
		EvaluationContext eval_context(next_state,  succ_g, true, &statistics);
		current_eval_context = eval_context;


		//Collect frontier states for refinement
		vector<GlobalState> frontier_states;
		while(!open_list->empty()){
			vector<int> keys;
			StateID id = open_list->remove_min(&keys);
			GlobalState s = state_registry->lookup_state(id);
			frontier_states.push_back(s);
		}

		//cout << "Frontier nodes: " << frontier_states.size() << endl;

		//refine heuristic on the next state
		//refine heuristic an all expanded sates
		cout << "+++++++++++++ REFINE ++++++++++++++++" << endl;
		//cout << "Number expanded states: " << expand_states.size() << endl;
		for(uint i = 1; i < expand_states.size(); i++){
			StateID refine_state_id = expand_states[i];
			cout << "----------> STATE: " << refine_state_id << endl;
			//cout << "Refine state: " << refine_state_id << endl;
			GlobalState refine_state = state_registry->lookup_state(refine_state_id);
			SearchNode node = search_space->get_node(refine_state);
			StateID parent_id = node.get_parent_id();
			//cout << "Parent: " << parent_id <<  " " << parent_id.hash() << endl;
			GlobalState parent_state = state_registry->lookup_state(parent_id);
			frontier_states.clear();
			frontier_states.push_back(parent_state);

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
		
		
			if(frontier_states.size() > 0){
				//heuristic->online_Refine(current_eval_context.get_state(), succStates, frontier_states);
				vector<pair<int,int>> pre_con;
				for(const GlobalCondition con : next_action->get_preconditions()){
					pre_con.push_back(make_pair(con.var, con.val));
				}
				heuristic->online_Refine(refine_state, succStates, frontier_states, pre_con);
			}
		}

		//only one step
		break;
	}

	cout << "+++++++++++++ REFINE ++++++++++++++++" << endl;
	//Reset search 
	search_space = new SearchSpace(*state_registry, cost_type);
	expand_states.clear();
	
	return IN_PROGRESS;
}

SearchStatus RealTimeSearch::step() {
	cout << "+++++++++++++ STEP +++++++++++++++" << endl;
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
    return  search();
}

SearchStatus RealTimeSearch::search() {
	int lookahead = 5;

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
		lookahead--;

		//If solution has been found or lookhead is reached return the current
		//best state (next min in openlist)
		bool solution_found = check_goal_and_set_plan(state);
		if((lookahead == 0 || solution_found)){
			expand_states.push_back(state.get_id());
			//cout << "----> compute next real time step" << endl;
			return compute_next_real_time_step(state, solution_found);
		}
		
		vector<const GlobalOperator *> applicable_ops;
    	g_successor_generator->generate_applicable_ops(state, applicable_ops);

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
		expand_states.push_back(state.get_id());
    }
    cout << "No solution - FAILED" << endl;
    return FAILED;
}

void RealTimeSearch::print_statistics() const {
    statistics.print_detailed_statistics();

    cout << "EHC phases: " << num_ehc_phases << endl;
    //assert(num_ehc_phases != 0);
    cout << "Average expansions per EHC phase: "
         << static_cast<double>(statistics.get_expanded()) / num_ehc_phases
         << endl;

    for (auto count : d_counts) {
        int depth = count.first;
        int phases = count.second.first;
        assert(phases != 0);
        int total_expansions = count.second.second;
        cout << "EHC phases of depth " << depth << ": " << phases
             << " - Avg. Expansions: "
             << static_cast<double>(total_expansions) / phases << endl;
    }

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
