#include "eager_search.h"

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

namespace eager_search {
EagerSearch::EagerSearch(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      use_multi_path_dependence(opts.get<bool>("mpd")),
      //Online Refinement ops
      refine_online(opts.get<bool>("refine_online")),
      use_min_h_value(opts.get<bool>("use_min_h_value")),
      refinement_threshold(opts.get<int>("refinement_threshold")),
      refinement_selector(opts.get<int>("refinement_selector")),
      //Store open list factory to create new open lists during search
      open_list_factory(opts.get<shared_ptr<OpenListFactory>>("open")),  
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
      f_evaluator(opts.get<ScalarEvaluator *>("f_eval", nullptr)),
      preferred_operator_heuristics(opts.get_list<Heuristic *>("preferred")),
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")),
      num_nodes_with_improvable_h_value(0) {
}

void EagerSearch::initialize() {
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

        
        ScalarEvaluator *heuristic = heuristics[0];
        int h = eval_context.get_heuristic_value_or_infinity(heuristic);
        //cout << "h=" << h << endl;
        node.set_h_value(h);
        open_list->insert(eval_context, initial_state.get_id());
    }

    print_initial_h_values(eval_context);

    pruning_method->initialize(g_root_task());
}

void EagerSearch::print_checkpoint_line(int g) const {
    cout << "[g=" << g << ", ";
    statistics.print_basic_statistics();
    cout << "]" << endl;
}

void EagerSearch::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    pruning_method->print_statistics();
    cout << "Nodes with improvable h values: " << num_nodes_with_improvable_h_value << endl;
    cout << "Nodes which have been refined: " << num_refined_nodes << endl;
    cout << "Nodes which have been improved: " << num_nodes_improved << endl;
    
    cout << endl;
    Heuristic *h = heuristics[0]; 
    h->print_statistics();
    cout << "Timer OpenList: " << open_list_timer << endl;
}

SearchStatus EagerSearch::step() {
    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
        return FAILED;
    }
    SearchNode node = n.first;

    GlobalState s = node.get_state();
    if (check_goal_and_set_plan(s))
        return SOLVED;

    vector<const GlobalOperator *> applicable_ops;
    g_successor_generator->generate_applicable_ops(s, applicable_ops);

    
    
    //------------------------- ONLINE REFINEMENT ----------------------------------------
    
    if(refine_online){
        bool debug = false;
        // Check whether h(s) is too low by looking at all successors.
        assert(heuristics.size() == 1);  // HACK
        ScalarEvaluator *heuristic = heuristics[0];  // HACK
        int infinity = EvaluationResult::INFTY;
        EvaluationContext state_eval_context(s, node.get_g(), false, nullptr);
        int state_h = state_eval_context.get_heuristic_value_or_infinity(heuristic);
        min_h_value = min(min_h_value, state_h);
        if(debug)
            cout << "h(s) = " << to_string(state_h) << endl;

        //compute provable_h_value
        int provable_h_value = infinity;
        if (state_h != infinity) {
            string succ_states_values("Succ h values:");
            for (const GlobalOperator *op : applicable_ops) {
                GlobalState succ_state = state_registry.get_successor_state(s, *op);
                int succ_g = node.get_g() + op->get_cost();
                EvaluationContext succ_eval_context(succ_state, succ_g, false, nullptr);
                int succ_h = succ_eval_context.get_heuristic_value_or_infinity(heuristic);
                succ_states_values +=  " " + to_string(succ_h);
                provable_h_value = min(
                    provable_h_value,
                    succ_h == infinity ? infinity : succ_h + op->get_cost());
            }
            if(debug){
             cout << succ_states_values << endl;
            }
        }
        assert(provable_h_value >= state_h);   

        //Check if refinement possible
        if (provable_h_value > state_h + refinement_threshold) {
            ++num_nodes_with_improvable_h_value;
            if(debug){
                cout << "--------------------------" << endl;
                cout << "g=" << node.get_g() << ", h improvable: " << state_h << " -> " << provable_h_value << endl;
            }    

			if(debug){
				//Check which heuristics should be refined
				vector<int> values = heuristics[0]->compute_individual_heuristics(s);
				string h_s("h(s)= ");
				for(uint i = 0; i < values.size(); i++){
					   h_s += to_string(values[i]);
					if(i < values.size() - 1){
						h_s += " + ";   
					}
				}
				if(debug){
					cout << h_s << endl;
				}
				vector<int> provable_h_values;
				//init
				for(uint i = 0; i < values.size(); i++){
					   provable_h_values.push_back(infinity);
				}
				for (const GlobalOperator *op : applicable_ops) {
					string succ_h_values("succ h values: ");
					GlobalState succ_state = state_registry.get_successor_state(s, *op);
					vector<int> succ_values = heuristics[0]->compute_individual_heuristics(succ_state);
					for(uint i = 0; i < provable_h_values.size(); i++){
						succ_h_values += to_string(succ_values[i]) + " ";
						provable_h_values[i] = min(
							provable_h_values[i],
							succ_values[i] == infinity ? infinity : succ_values[i] + op->get_cost());
					}
					if(debug)
						cout << succ_h_values << endl;
				}
				bool conflict = true;
				string provable_h_values_s("provable h values: ");
				for(uint i = 0; i < provable_h_values.size(); i++){
					provable_h_values_s += to_string(provable_h_values[i]) + " ";
					if(provable_h_values[i] >= values[i]){
						conflict = false;
						provable_h_values_s += "r ";   
					}
				}

				if(debug){
					cout << provable_h_values_s << endl;
					cout <<"refinement conflict: " << conflict << endl;
				}
			}

            //cout << state_h << " <=> " << min_h_value << " " << endl;
            bool refine_min_h_value = true;
            if(use_min_h_value){
                refine_min_h_value = state_h == min_h_value;
            }
            if(refine_min_h_value && num_nodes_with_improvable_h_value % refinement_selector == 0){
                
                if(debug)    
                    cout << "old h value: "  << state_h << endl;

                //ONLINE REFINEMENT    
                Heuristic *h = heuristics[0]; 
                //refine_timer.resume();
                bool refined = h->online_Refine(s);
                //refine_timer.stop();
                //cout << "Refine Timer: " << refine_timer << endl;
                

                
                    //reevaluate cached values
                    auto &cached_result = const_cast<HeuristicCache &>(state_eval_context.get_cache())[heuristic];
                    if (!cached_result.is_uninitialized()){
                        cached_result = EvaluationResult();
                    }
                    //cout << "refine from " << state_h << " to "; 
                    int new_h_value = state_eval_context.get_heuristic_value_or_infinity(heuristic);
                    if(new_h_value > state_h){
                        num_nodes_improved++;   
                    }
                    state_h = new_h_value;
                    //cout << state_h << endl;
                    min_h_value = state_h;
                if(debug){
                    cout << "new h value: " << state_h << endl;
                }


                if(refined){
                    num_refined_nodes++;
                   
                    //Update optenlist
                    /*
                    if(num_refined_nodes % 100 == 0){                       
                        open_list_timer.resume();
                        //cout << "Update openlist " << num_refined_nodes << endl;
                        std::unique_ptr<StateOpenList> new_open_list = open_list_factory->create_state_open_list();
                        int size_openList = 0;
                        while(!open_list->empty()){
                            StateID id = open_list->remove_min(nullptr);
                            GlobalState s = state_registry.lookup_state(id);
                            SearchNode node = search_space.get_node(s);
                            EvaluationContext eval_context(node.get_state(), node.get_g(), false, &statistics);
                            new_open_list->insert(eval_context, node.get_state_id());      
                            size_openList++;
                        }

                        open_list.reset(new_open_list.release()); //TODO unique_ptr 
                        open_list_timer.stop();    
                        if(print_timer() > 30){
                            cout << "OpenList Timer: " << open_list_timer << " size: " << size_openList << endl;   
                            print_timer.reset();
                        }
                    }*/
                }  
                
            }
            if(debug)
                cout << "+++++++++++++++++++++++++++++++" << endl;
        }
    }
    //------------------------- ONLINE REFINEMENT ----------------------------------------
    
    
    /*
      TODO: When preferred operators are in use, a preferred operator will be
      considered by the preferred operator queues even when it is pruned.
    */
    pruning_method->prune_operators(s, applicable_ops);

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
            
            
            //Store old h value
            ScalarEvaluator *heuristic = heuristics[0];
            int h = eval_context.get_heuristic_value_or_infinity(heuristic);
            //cout << "h=" << h << endl;
            succ_node.set_h_value(h);

            open_list->insert(eval_context, succ_state.get_id());
            if (search_progress.check_progress(eval_context)) {
                print_checkpoint_line(succ_node.get_g());
                reward_progress();
            }
        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
                if (succ_node.is_closed()) {
                    /*
                      TODO: It would be nice if we had a way to test
                      that reopening is expected behaviour, i.e., exit
                      with an error when this is something where
                      reopening should not occur (e.g. A* with a
                      consistent heuristic).
                    */
                    statistics.inc_reopened();
                }
                succ_node.reopen(node, op);

                EvaluationContext eval_context(
                    succ_state, succ_node.get_g(), is_preferred, &statistics);

                /*
                  Note: our old code used to retrieve the h value from
                  the search node here. Our new code recomputes it as
                  necessary, thus avoiding the incredible ugliness of
                  the old "set_evaluator_value" approach, which also
                  did not generalize properly to settings with more
                  than one heuristic.

                  Reopening should not happen all that frequently, so
                  the performance impact of this is hopefully not that
                  large. In the medium term, we want the heuristics to
                  remember heuristic values for states themselves if
                  desired by the user, so that such recomputations
                  will just involve a look-up by the Heuristic object
                  rather than a recomputation of the heuristic value
                  from scratch.
                */
                open_list->insert(eval_context, succ_state.get_id());
            } else {
                // If we do not reopen closed nodes, we just update the parent pointers.
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back.
                succ_node.update_parent(node, op);
            }
        }
    }

    return IN_PROGRESS;
}

pair<SearchNode, bool> EagerSearch::fetch_next_node() {
    /* TODO: The bulk of this code deals with multi-path dependence,
       which is a bit unfortunate since that is a special case that
       makes the common case look more complicated than it would need
       to be. We could refactor this by implementing multi-path
       dependence as a separate search algorithm that wraps the "usual"
       search algorithm and adds the extra processing in the desired
       places. I think this would lead to much cleaner code. */

    open_list_timer.resume();
    while (true) {
        if (open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;
            // HACK! HACK! we do this because SearchNode has no default/copy constructor
            const GlobalState &initial_state = state_registry.get_initial_state();
            SearchNode dummy_node = search_space.get_node(initial_state);
            return make_pair(dummy_node, false);
        }
        vector<int> last_key_removed;
        StateID id = open_list->remove_min(
            use_multi_path_dependence ? &last_key_removed : nullptr);
        // TODO is there a way we can avoid creating the state here and then
        //      recreate it outside of this function with node.get_state()?
        //      One way would be to store GlobalState objects inside SearchNodes
        //      instead of StateIDs
        GlobalState s = state_registry.lookup_state(id);
        SearchNode node = search_space.get_node(s);
        
        //Check if state needs to be reevaluated        
        if(true){
            int old_h = node.get_h_value();
            
            EvaluationContext state_eval_context(s, node.get_g(), false, nullptr);
            ScalarEvaluator *heuristic = heuristics[0];
            int new_h = state_eval_context.get_heuristic_value_or_infinity(heuristic);

            //cout << old_h << " " << new_h << endl;
            if(old_h != new_h){
                node.set_h_value(new_h);
                open_list->insert(state_eval_context, node.get_state_id());  
                num_reeval_states++;
                //cout << "Num reeval states " << num_reeval_states << endl; 
                continue;  
            }
            
            open_list_timer.stop();
            if(print_timer() > 60){
                cout << "Num reeval states " << num_reeval_states  << " OpenList Timer: " << open_list_timer << endl;   
                print_timer.reset();
                //print_statistics();
            }
        }
        
        
        if (node.is_closed())
            continue;

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

void EagerSearch::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void EagerSearch::dump_search_space() const {
    search_space.dump();
}

void EagerSearch::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_heuristic_value(f_evaluator);
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void EagerSearch::update_f_value_statistics(const SearchNode &node) {
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

static SearchEngine *_parse(OptionParser &parser) {
    parser.document_synopsis("Eager best-first search", "");

    parser.add_option<shared_ptr<OpenListFactory>>("open", "open list");
    parser.add_option<bool>("reopen_closed",
                            "reopen closed nodes", "false");
    parser.add_option<ScalarEvaluator *>(
        "f_eval",
        "set evaluator for jump statistics. "
        "(Optional; if no evaluator is used, jump statistics will not be displayed.)",
        OptionParser::NONE);
    parser.add_list_option<Heuristic *>(
        "preferred",
        "use preferred operators of these heuristics", "[]");

    add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    EagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        opts.set<bool>("mpd", false);
        engine = new EagerSearch(opts);
    }

    return engine;
}

static SearchEngine *_parse_astar(OptionParser &parser) {
    parser.document_synopsis(
        "A* search (eager)",
        "A* is a special case of eager best first search that uses g+h "
        "as f-function. "
        "We break ties using the evaluator. Closed nodes are re-opened.");
    parser.document_note(
        "mpd option",
        "This option is currently only present for the A* algorithm and not "
        "for the more general eager search, "
        "because the current implementation of multi-path depedence "
        "does not support general open lists.");
    parser.document_note(
        "Equivalent statements using general eager search",
        "\n```\n--search astar(evaluator)\n```\n"
        "is equivalent to\n"
        "```\n--heuristic h=evaluator\n"
        "--search eager(tiebreaking([sum([g(), h]), h], unsafe_pruning=false),\n"
        "               reopen_closed=true, f_eval=sum([g(), h]))\n"
        "```\n", true);
    parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");
    parser.add_option<bool>("mpd",
                            "use multi-path dependence (LM-A*)", "false");
    
    //Online Refinment options
    parser.add_option<bool>(
        "refine_online",
        "use online refinement",
        "false");
    parser.add_option<bool>(
        "use_min_h_value",
        "only refine a variable if its h value is smaller or equal to the current minimum",
        "false");
    parser.add_option<int>(
        "refinement_threshold",
        "the threshold the provable minimum h value has to exceed to start online refinement",
        "0",
        Bounds("0", "infinity"));
    parser.add_option<int>(
        "refinement_selector",
        "only every refinement_selector states is refined",
        "1",
        Bounds("1", "10000"));

    add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    EagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
        opts.set("open", temp.first);
        opts.set("f_eval", temp.second);
        opts.set("reopen_closed", true);
        vector<Heuristic *> preferred_list;
        opts.set("preferred", preferred_list);
        engine = new EagerSearch(opts);
    }

    return engine;
}

static SearchEngine *_parse_greedy(OptionParser &parser) {
    parser.document_synopsis("Greedy search (eager)", "");
    parser.document_note(
        "Open list",
        "In most cases, eager greedy best first search uses "
        "an alternation open list with one queue for each evaluator. "
        "If preferred operator heuristics are used, it adds an extra queue "
        "for each of these evaluators that includes only the nodes that "
        "are generated with a preferred operator. "
        "If only one evaluator and no preferred operator heuristic is used, "
        "the search does not use an alternation open list but a "
        "standard open list with only one queue.");
    parser.document_note(
        "Closed nodes",
        "Closed node are not re-opened");
    parser.document_note(
        "Equivalent statements using general eager search",
        "\n```\n--heuristic h2=eval2\n"
        "--search eager_greedy([eval1, h2], preferred=h2, boost=100)\n```\n"
        "is equivalent to\n"
        "```\n--heuristic h1=eval1 --heuristic h2=eval2\n"
        "--search eager(alt([single(h1), single(h1, pref_only=true), single(h2), \n"
        "                    single(h2, pref_only=true)], boost=100),\n"
        "               preferred=h2)\n```\n"
        "------------------------------------------------------------\n"
        "```\n--search eager_greedy([eval1, eval2])\n```\n"
        "is equivalent to\n"
        "```\n--search eager(alt([single(eval1), single(eval2)]))\n```\n"
        "------------------------------------------------------------\n"
        "```\n--heuristic h1=eval1\n"
        "--search eager_greedy(h1, preferred=h1)\n```\n"
        "is equivalent to\n"
        "```\n--heuristic h1=eval1\n"
        "--search eager(alt([single(h1), single(h1, pref_only=true)]),\n"
        "               preferred=h1)\n```\n"
        "------------------------------------------------------------\n"
        "```\n--search eager_greedy(eval1)\n```\n"
        "is equivalent to\n"
        "```\n--search eager(single(eval1))\n```\n", true);

    parser.add_list_option<ScalarEvaluator *>("evals", "scalar evaluators");
    parser.add_list_option<Heuristic *>(
        "preferred",
        "use preferred operators of these heuristics", "[]");
    parser.add_option<int>(
        "boost",
        "boost value for preferred operator open lists", "0");

    add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);

    Options opts = parser.parse();
    opts.verify_list_non_empty<ScalarEvaluator *>("evals");

    EagerSearch *engine = nullptr;
    if (!parser.dry_run()) {
        opts.set("open", search_common::create_greedy_open_list_factory(opts));
        opts.set("reopen_closed", false);
        opts.set("mpd", false);
        ScalarEvaluator *evaluator = nullptr;
        opts.set("f_eval", evaluator);
        engine = new EagerSearch(opts);
    }
    return engine;
}

static Plugin<SearchEngine> _plugin("eager", _parse);
static Plugin<SearchEngine> _plugin_astar("astar", _parse_astar);
static Plugin<SearchEngine> _plugin_greedy("eager_greedy", _parse_greedy);
}
