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
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")){		  
}

void DFSPruning::initialize() {
    
    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
    //Stop Timer
	open_list_timer.reset();
    open_list_timer.stop();
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
    cout << "Nodes which have been refined: " << num_refined_nodes << endl;
    cout << "Number of  reevaluated states: " << num_reeval_states << endl;
	
	cout << "openlist time: " << open_list_timer << endl;
    cout << "total refine time: " << total_refine_timer << endl;
    cout << endl;	
    
    cout << endl;
    Heuristic *h = heuristics[0]; 
    h->print_statistics();
    
    
}

SearchStatus DFSPruning::step() {
	cout << "--------------------------------------------------------" << endl;
    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
        return FAILED;
    }
    SearchNode node = n.first;
	
	cout << "Expanded node " << node.get_state_id() << endl;

    GlobalState s = node.get_state();
    if (check_goal_and_set_plan(s))
        return SOLVED;

    vector<const GlobalOperator *> applicable_ops;
    g_successor_generator->generate_applicable_ops(s, applicable_ops);

    
    /*
      TODO: When preferred operators are in use, a preferred operator will be
      considered by the preferred operator queues even when it is pruned.
    */
    pruning_method->prune_operators(s, applicable_ops);

    // This evaluates the expanded state (again) to get preferred ops
    EvaluationContext eval_context(s, node.get_g(), false, &statistics, true);
    algorithms::OrderedSet<const GlobalOperator *> preferred_operators =
        collect_preferred_operators(eval_context, preferred_operator_heuristics);

	
	//Find node with the smallest h value
	//int min_h = EvaluationResult::INFTY;
    for (const GlobalOperator *op : applicable_ops) {
        if ((node.get_real_g() + op->get_cost()) >= bound)
            continue;

		
        GlobalState succ_state = state_registry.get_successor_state(s, *op);
        statistics.inc_generated();
        bool is_preferred = preferred_operators.contains(op);

        SearchNode succ_node = search_space.get_node(succ_state);
		
		cout << "Child " << succ_node.get_state_id() << endl; 

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

        if (succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.

            // Careful: succ_node.get_g() is not available here yet,
            // hence the stupid computation of succ_g.
            // TODO: Make this less fragile.
            int succ_g = node.get_g() + get_adjusted_cost(*op);

            EvaluationContext eval_context(
                succ_state, succ_g, is_preferred, &statistics);
            statistics.inc_evaluated_states();

            if (open_list->is_dead_end(eval_context)) {
                succ_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }
            succ_node.open(node, op);
            
            
            //Store h value in node for online refinement openlist check
            ScalarEvaluator *heuristic = heuristics[0];
            int h = eval_context.get_heuristic_value_or_infinity(heuristic);
            succ_node.set_h_value(h);
			
			//find new min
			/*
			cout << "h = " << h << endl;
			if(h < min_h){
				min_h = h;
				cout << "new min h value: " << min_h << " State: " << succ_node.get_state_id() << endl;
				currentStateID = succ_node.get_state_id();
			}
			*/
            open_list->insert(eval_context, succ_state.get_id());
			
            if (search_progress.check_progress(eval_context)) {
                print_checkpoint_line(succ_node.get_g());
                reward_progress();
            }
        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
                if (succ_node.is_closed()) {                    
                    
                    statistics.inc_reopened();
                }
                succ_node.reopen(node, op);

                EvaluationContext eval_context(
                    succ_state, succ_node.get_g(), is_preferred, &statistics);             
              
                open_list->insert(eval_context, succ_state.get_id());
            } else {
                
                succ_node.update_parent(node, op);
            }
        }
    }

    return IN_PROGRESS;
}

pair<SearchNode, bool> DFSPruning::fetch_next_node() {
	/*
	cout << "fetch_next_node" << endl;
		GlobalState sn = state_registry.lookup_state(currentStateID);
        SearchNode currentNode = search_space.get_node(sn);
		cout << "Current Node: " << currentNode.get_state_id() << endl;
		vector<const GlobalOperator *> applicable_ops;
		g_successor_generator->generate_applicable_ops(currentNode.get_state(), applicable_ops);

		if(applicable_ops.size() > 0){
			return make_pair(currentNode, true);	
		}
	*/
	
	cout << "---> open_list node" << endl;
    while (true) {
        if (open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;
            // HACK! HACK! we do this because SearchNode has no default/copy constructor
            const GlobalState &initial_state = state_registry.get_initial_state();
            SearchNode dummy_node = search_space.get_node(initial_state);
            return make_pair(dummy_node, false);
        }
        vector<int> last_key_removed;
        //use the last_key_removed if the position in the openlist and the and h and g
        //values of the state match
        StateID id = open_list->remove_min(&last_key_removed);
        // TODO is there a way we can avoid creating the state here and then
        //      recreate it outside of this function with node.get_state()?
        //      One way would be to store GlobalState objects inside SearchNodes
        //      instead of StateIDs
        GlobalState s = state_registry.lookup_state(id);
        SearchNode node = search_space.get_node(s);       
        
        if (node.is_closed())
            continue;
        
        /*
        if(print_timer() > 60){
            cout << "+++++++++++++++++++++++++++++++++++++" << endl;                       
            cout << "Num reeval states " << num_reeval_states  << endl;
			cout << "OpenList Timer: " << open_list_timer << endl << endl;   
            print_statistics();
			print_timer.reset();
        }
		*/

        if (use_multi_path_dependence) {
            assert(last_key_removed.size() == 2);
            if (node.is_dead_end())
                continue;
            int pushed_h = last_key_removed[1];

            if (!node.is_closed()) {
                EvaluationContext eval_context(
                    node.get_state(), node.get_g(), false, &statistics);

                if (open_list->is_dead_end(eval_context)) {
                    node.mark_as_dead_end();
                    statistics.inc_dead_ends();
                    continue;
                }
                if (pushed_h < eval_context.get_result(heuristics[0]).get_h_value()) {
                    assert(node.is_open());                   
                    open_list->insert(eval_context, node.get_state_id());
                    continue;
                }
            }
        }
        

        node.close();
        assert(!node.is_dead_end());
        update_f_value_statistics(node);
        statistics.inc_expanded();
        return make_pair(node, true);
    }
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
        "1",
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
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
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
