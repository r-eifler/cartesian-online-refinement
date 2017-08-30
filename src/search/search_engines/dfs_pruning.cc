#include "dfs_pruning.h"

#include "search_common.h"

#include "../evaluation_context.h"
#include "../globals.h"
#include "../heuristic.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../pruning_method.h"
#include "../successor_generator.h"

#include "../algorithms/ordered_set.h"

#include "../open_lists/open_list_factory.h"

#include "../cegar/additive_cartesian_heuristic.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <set>
#include <string>

using namespace std;

namespace dfs_pruning {
DFSPruning::DFSPruning(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      use_multi_path_dependence(opts.get<bool>("mpd")),
      //Online Refinement ops
      refine_online(opts.get<bool>("refine_online")),
      refinement_selector(opts.get<int>("refinement_selector")),
	  refinement_time(opts.get<double>("refinement_time")),
	  collect_states(opts.get<int>("collect_states")),  
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
      f_evaluator(opts.get<ScalarEvaluator *>("f_eval", nullptr)),
      preferred_operator_heuristics(opts.get_list<Heuristic *>("preferred")),
	  pruning_heuristic(opts.get<ScalarEvaluator *>("pruningh", nullptr)),
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")){		  
}

void DFSPruning::initialize() {
    
    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
    //Stop Timer
	total_refine_timer.reset();
    total_refine_timer.stop();
    
    
    cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
    if (use_multi_path_dependence)
        cout << "Using multi-path dependence (LM-A*)" << endl;
    assert(open_list);

    set<Heuristic *> hset;
    open_list->get_involved_heuristics(hset);

    // add heuristics that are used for preferred operators (in case they are
    // not also used in the open list)
    hset.insert(preferred_operator_heuristics.begin(),
                preferred_operator_heuristics.end());

    // add heuristics that are used in the f_evaluator. They are usually also
    // used in the open list and hence already be included, but we want to be
    // sure.
    if (f_evaluator) {
        f_evaluator->get_involved_heuristics(hset);
    }

    heuristics.assign(hset.begin(), hset.end());
    assert(!heuristics.empty());

    const GlobalState &initial_state = state_registry.get_initial_state();
    for (Heuristic *heuristic : heuristics) {
        heuristic->notify_initial_state(initial_state);
    }

    // Note: we consider the initial state as reached by a preferred
    // operator.
    EvaluationContext eval_context(initial_state, 0, true, &statistics);

    statistics.inc_evaluated_states();

    if (open_list->is_dead_end(eval_context)) {
        cout << "Initial state is a dead end." << endl;
    } else {
        if (search_progress.check_progress(eval_context))
            print_checkpoint_line(0);
        start_f_value_statistics(eval_context);
        SearchNode node = search_space.get_node(initial_state);
        node.open_initial();
		
        
        //Initialize the heuristc value of the serach node such that it can be checked in the 
		//fetch_next_node function
        ScalarEvaluator *heuristic = heuristics[0];
        int h = eval_context.get_heuristic_value_or_infinity(heuristic);
        node.set_h_value(h);
		node.set_depth(0);
        open_list->insert(eval_context, initial_state.get_id());
    }

    print_initial_h_values(eval_context);

    pruning_method->initialize(g_root_task());
}

void DFSPruning::print_checkpoint_line(int g) const {
    cout << "[g=" << g << ", ";
    statistics.print_basic_statistics();
    cout << "]" << endl;
}

void DFSPruning::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    pruning_method->print_statistics();
    
    cout << endl;
	cout << "Number of pruned states: " << num_pruned_states << endl;
	cout << "Number of refined states: " << num_refined_states << endl;
	
	cout << endl << "Current upper bound: " << get_plan_cost() << endl;
	cout << "Update steps: " << update_steps << endl;
    cout << "total refine time: " << total_refine_timer << endl;
    cout << endl;	
    
    cout << endl;
    Heuristic* h = (Heuristic*) pruning_heuristic; 
    h->print_statistics();
    
    
}

SearchStatus DFSPruning::step() {
	//cout << "--------------------------------------------------------" << endl;
    pair<SearchNode, int> n = fetch_next_node();
    if (n.second == 0) {
        return FAILED;
    }
	//The whole state space has been explored -> current solution optimal
	if(n.second == 2){
		cout << "Solution Found!" << endl;
		check_goal_and_set_plan(*current_goal_state);
		return SOLVED;	
	}

    SearchNode node = n.first;

    GlobalState s = node.get_state();
	current_path.push_back(s);
	/*
	cout << "Current Path: ";
	for(GlobalState cs : current_path){
		cout << search_space.get_node(cs).get_state_id() << " ";	
	}
	cout << endl;
	*/
	
	//Check if solution found
    if (check_goal_and_set_plan(s)){
		if(open_list->empty()){ // can this case appeare ?
        	return SOLVED;
		}
		//check if new solution is cheaper -> update
		if((int) get_plan_cost() < upper_bound){
			update_steps++;
			if(upper_bound == EvaluationResult::INFTY){
				cout << "First upper bound: " << 	get_plan_cost() << endl;
			}
			//ut << "Upper bound updated ----> " << upper_bound << " -> " << get_plan_cost() << endl;
			upper_bound = get_plan_cost(); 
			current_goal_state = &s;
			/*
			for(GlobalState cs : current_path){
				cout << search_space.get_node(cs).get_state_id() << " ";	
			}
			cout << endl;
			*/
			for(GlobalState cs : current_solution){
				SearchNode cs_node = search_space.get_node(cs);
				cs_node.set_solution(false);
			}
			
			//TODO can I use =
			current_solution.clear();
			for(GlobalState cs : current_path){
				current_solution.push_back(cs);
			}
				
			
			for(GlobalState cs : current_solution){
				SearchNode cs_node = search_space.get_node(cs);
				cs_node.set_solution(true);
			}
		}
		/*
		else{
			cout << "Solution worse ----> " << upper_bound << " -> " << get_plan_cost() << endl;
		}
		*/
		//cout << "Look for better solution" << endl;
		
		return IN_PROGRESS;
	}
	
    vector<const GlobalOperator *> applicable_ops;
    g_successor_generator->generate_applicable_ops(s, applicable_ops);

    
    /*
      TODO: When preferred operators are in use, a preferred operator will be
      considered by the preferred operator queues even when it is pruned.
    */
    //pruning_method->prune_operators(s, applicable_ops);

    // This evaluates the expanded state (again) to get preferred ops
    EvaluationContext eval_context(s, node.get_g(), false, &statistics, true);
    algorithms::OrderedSet<const GlobalOperator *> preferred_operators =
        collect_preferred_operators(eval_context, preferred_operator_heuristics);

	
    for (const GlobalOperator *op : applicable_ops) {
        if ((node.get_real_g() + op->get_cost()) >= bound)
            continue;

		
		
        GlobalState succ_state = state_registry.get_successor_state(s, *op);
        statistics.inc_generated();
        bool is_preferred = preferred_operators.contains(op);

        SearchNode succ_node = search_space.get_node(succ_state);
		//cout << "Child node: " << succ_node.get_state_id() << endl;
		
        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end())
            continue;

        // update new path
        if (use_multi_path_dependence || succ_node.is_new()) {
            /*
              Note: we must call notify_state_transition for each heuristic, so
              don't break out of the for loop early.
            */
            for (Heuristic *heuristic : heuristics) {
                heuristic->notify_state_transition(s, *op, succ_state);
            }
        }
		
		
		// Careful: succ_node.get_g() is not available here yet,
        // hence the stupid computation of succ_g.
        // TODO: Make this less fragile.
        int succ_g = node.get_g() + get_adjusted_cost(*op);

		//PRUN
		int h = eval_context.get_heuristic_value_or_infinity(pruning_heuristic);
		succ_node.set_h_value(h);
		//cout << "	Prune: " << succ_g << " + " << h << " = " << (succ_g + h) <<  " > " << upper_bound << endl;
		//need to compute succ.g
		if(succ_g + h > upper_bound){
			//cout << "		----> Prune" << endl;
			num_pruned_states++;
			continue;	
		}
		
			
		
        if (succ_node.is_new()) {
			//set depth if node is new
			succ_node.set_depth(node.get_depth() + 1);
			
            // We have not seen this state before.
            // Evaluate and create a new node.            

            EvaluationContext eval_context(
                succ_state, succ_g, is_preferred, &statistics);
            statistics.inc_evaluated_states();

            if (open_list->is_dead_end(eval_context)) {
                succ_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }
            succ_node.open(node, op);
            

            open_list->insert(eval_context, succ_state.get_id());
			openlist_size++;
			
            if (search_progress.check_progress(eval_context)) {
                print_checkpoint_line(succ_node.get_g());
                reward_progress();
            }
        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
			//set depth if a chaeper path to the node has been found
			succ_node.set_depth(node.get_depth() + 1);
			
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
                if (succ_node.is_closed()) {                    
                    
                    statistics.inc_reopened();
                }
                succ_node.reopen(node, op);

                EvaluationContext eval_context(
                    succ_state, succ_node.get_g(), is_preferred, &statistics);             
              
                open_list->insert(eval_context, succ_state.get_id());
				openlist_size++;
            } else {
                
                succ_node.update_parent(node, op);
            }
        }

    }

    return IN_PROGRESS;
}

pair<SearchNode, int> DFSPruning::fetch_next_node() {	
    while (true) {
        if (open_list->empty()){
			const GlobalState &initial_state = state_registry.get_initial_state();
			SearchNode dummy_node = search_space.get_node(initial_state);
			if (current_goal_state == NULL) {
				cout << "Completely explored state space -- no solution!" << endl;
				return make_pair(dummy_node, 0);
			}
			else{
				cout << "Completely explored state space -- solution found!" << endl;
				return make_pair(dummy_node, 2);
			}
		}
        vector<int> last_key_removed;
        //the first position in last_key_removed indicates if there was a backtrack
		
        StateID id = open_list->remove_min(&last_key_removed);
		openlist_size--;
        
        GlobalState s = state_registry.lookup_state(id);
        SearchNode node = search_space.get_node(s);       
		
		//if there is no online refinement the current path has to be updated seperately
		if(!refine_online && last_key_removed[0]){
			current_path.erase(current_path.end() - (current_path.size() - node.get_depth()), current_path.end());
		}
		
		 if (node.is_closed())
            continue;
		
		//cout << "Expanded node: " << node.get_state_id() << endl;		
		//Refine the pruning heuristic such the state space area which did not lead to a better solution is pruned
		if(refine_online){
			refine(last_key_removed[0], node.get_depth());
		}

		//The value of the pruning heuristic could have changed since the node has been inserted in the open list 
		//or a new path could be found (-> also in offline version)
		EvaluationContext state_eval_context(s, node.get_g(), false, nullptr);
		int state_h = state_eval_context.get_heuristic_value_or_infinity(pruning_heuristic);
		if(node.get_g() + state_h > upper_bound){
			/*
			cout << "	Prune: " << node.get_g() << " + " << state_h << " = " << (node.get_g() + state_h) <<  " > " << upper_bound << endl;
			cout << "	----> Prune fetch" << endl;
			cout << "......................" << endl;
			*/
			num_pruned_states++;
			continue;	
		}
		/*
        if(print_timer() > 10){
            cout << "+++++++++++++++++++++++++++++++++++++" << endl;                        
            print_statistics();
			print_timer.reset();
        }
		*/
        
		//no multipath dependencies
		
        node.close();
        assert(!node.is_dead_end());
        update_f_value_statistics(node);
        statistics.inc_expanded();	
        return make_pair(node, 1);
    }
}
	
void DFSPruning::refine(bool backtracked, int backtrack_depth){
	total_refine_timer.resume();
	//cout << "backtracked: " << backtracked << endl;
	if(!backtracked || upper_bound == EvaluationResult::INFTY){
		total_refine_timer.stop();
		return;	
	}
	/*
	cout << "--- Backtrack ---" << endl;
	cout << "Current path lenght: " << current_path.size() << endl;
	cout << "Bacltrack depth: " << backtrack_depth << endl;
	//remove states from path	
	for(GlobalState cs : current_path){
		cout << search_space.get_node(cs).get_state_id() << " ";	
	}
	cout << endl;
	cout << "Remove: " << current_path.size() << " - " << backtrack_depth << "  " << (current_path.size() - backtrack_depth) << endl;
	*/
	//Backtrack the current path to the depth of the expanded node
	current_path.erase(current_path.end() - (current_path.size() - backtrack_depth - 1), current_path.end());
	GlobalState state_to_refine = *(current_path.end()-1);
	SearchNode last_node = search_space.get_node(state_to_refine);
	current_path.erase(current_path.end());
	/*
	for(GlobalState cs : current_path){
		cout << search_space.get_node(cs).get_state_id() << " ";	
	}
	cout << endl;	
	cout << "Refined state: " << last_node.get_state_id() << endl;
	*/
	//if the current node is contained in the current best solution it is not pruned
	if(last_node.get_solution()){ //|| statistics.get_expanded() % 1 != 0){
		//cout << "Countained in current solution" << endl;
		total_refine_timer.stop();
		return;	
	}		

	//cout << "------------------------- ONLINE REFINEMENT ----------------------------------------" << endl;
	
	vector<const GlobalOperator *> applicable_ops;
	g_successor_generator->generate_applicable_ops(state_to_refine, applicable_ops);
	  
	
	int infinity = EvaluationResult::INFTY;
	EvaluationContext state_eval_context(state_to_refine, last_node.get_g(), false, nullptr);
	int state_h = state_eval_context.get_heuristic_value_or_infinity(pruning_heuristic);

	if (state_h == infinity) {
		total_refine_timer.stop();
		return;
	}
	
	//Generate all succesor states 
	vector<pair<GlobalState, int>> succStates;
	/*
	for (const GlobalOperator *op : applicable_ops) {
		GlobalState succ_state = state_registry.get_successor_state(state_to_refine, *op);
		succStates.push_back(make_pair(succ_state, op->get_cost()));
	}
	*/
	//ONLINE REFINEMENT  
	Heuristic* h = (Heuristic*) pruning_heuristic;
	//cout << "Prune: " << last_node.get_g() << " + " << state_h << " = " << (last_node.get_g() + state_h) <<  " > " << upper_bound << endl; 
	if((last_node.get_g() + state_h) > upper_bound){
		//cout << "----> would already be pruned" << endl;
		total_refine_timer.stop();
		return;	
	}
	//upper_bound - last_node.get_g() is the value the heuristic has to exceed such that the state can be pruned
	h->online_Refine(state_to_refine, succStates, upper_bound - last_node.get_g());
	num_refined_states++;
	
	total_refine_timer.stop();
	//cout << "------------------------- ONLINE REFINEMENT END----------------------------------------" << endl;		
}

void DFSPruning::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void DFSPruning::dump_search_space() const {
    search_space.dump();
}

void DFSPruning::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_heuristic_value(f_evaluator);
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void DFSPruning::update_f_value_statistics(const SearchNode &node) {
    if (f_evaluator) {
        /*
          TODO: This code doesn't fit the idea of supporting
          an arbitrary f evaluator.
        */
        EvaluationContext eval_context(node.get_state(), node.get_g(), false, &statistics);
        int f_value = eval_context.get_heuristic_value(f_evaluator);
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: merge this into SearchEngine::add_options_to_parser when all search
         engines support pruning. */
void add_pruning_option(OptionParser &parser) {
    parser.add_option<shared_ptr<PruningMethod>>(
        "pruning",
        "Pruning methods can prune or reorder the set of applicable operators in "
        "each state and thereby influence the number and order of successor states "
        "that are considered.",
        "null()");
}


static SearchEngine *_parse_dfs(OptionParser &parser) {
    parser.document_synopsis(
        "DFS", "TODO");
    parser.document_note(
        "mpd option",
        "This option is currently only present for the A* algorithm and not "
        "for the more general TODO search, "
        "because the current implementation of multi-path depedence "
        "does not support general open lists.");
    parser.document_note(
        "Equivalent statements using general TODO search",
        "\n```\n--search astar(evaluator)\n```\n"
        "is equivalent to\n"
        "```\n--heuristic h=evaluator\n"
        "--search TODO(tiebreaking([sum([g(), h]), h], unsafe_pruning=false),\n"
        "               reopen_closed=true, f_eval=sum([g(), h]))\n"
        "```\n", true);
    parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");
	parser.add_option<ScalarEvaluator *>("pruningh", "evaluator for pruning");
    parser.add_option<bool>("mpd",
                            "use multi-path dependence (LM-A*)", "false");
    
    //Online Refinment options
    parser.add_option<bool>(
        "refine_online",
        "use online refinement",
        "true");
    parser.add_option<int>(
        "refinement_threshold",
        "the threshold the provable minimum h value has to exceed to start online refinement",
        "0",
        Bounds("0", "infinity"));
    parser.add_option<int>(
        "refinement_selector",
        "only every refinement_selector states is refined",
        "1",
        Bounds("1", "1000000"));
	parser.add_option<double>(
        "refinement_time",
        "only every refinement_times secondes a state is refined",
        "0",
        Bounds("0", "60"));
	parser.add_option<int>(
        "collect_states",
        "TODO",
        "1",
        Bounds("1", "100"));

    add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    DFSPruning *engine = nullptr;
    if (!parser.dry_run()) {
        auto temp = search_common::create_dfs_openlist(opts);
        opts.set("open", temp.first);
        opts.set("f_eval", temp.second);
        opts.set("reopen_closed", true);
        vector<Heuristic *> preferred_list;
        opts.set("preferred", preferred_list);
        engine = new DFSPruning(opts);
    }

    return engine;
}



static Plugin<SearchEngine> _plugin_dfs("dfs", _parse_dfs);
}
