#include "general_eager_best_first_search.h"

#include "globals.h"
#include "heuristic.h"
#include "successor_generator.h"

#include <cassert>
#include <cstdlib>
using namespace std;

GeneralEagerBestFirstSearch::GeneralEagerBestFirstSearch(bool reopen_closed):
    reopen_closed_nodes(reopen_closed),
    open_list(0), f_evaluator(0) {
}

GeneralEagerBestFirstSearch::~GeneralEagerBestFirstSearch() {
}

// TODO: changes this to add_open_list,
// including type of open list, use of preferred operators, which heuristic to use, etc.
void GeneralEagerBestFirstSearch::add_heuristic(Heuristic *heuristic,
  bool use_estimates,
  bool use_preferred_operators) {

    if (!use_estimates && !use_preferred_operators) {
        cerr << "WTF" << endl;
    }

    if (use_preferred_operators) {
        cerr << "Preferred operator not supported" << endl;
    }

    assert(use_estimates || use_preferred_operators);

    heuristics.push_back(heuristic);
    search_progress.add_heuristic(heuristic);
}

void GeneralEagerBestFirstSearch::initialize() {
    g_learning_search_space = &search_space; //TODO:CR - check if we can get of this
    if (heuristics.size() > 1) {
        cout << "WARNING: currently only one heuristic allowed in general search; ";
        cout << "skipping additional heuristics." << endl;
    }
    //TODO children classes should output which kind of search
    cout << "Conducting best first search" <<
        (reopen_closed_nodes? " with" : " without") << " reopening closes nodes" << endl;

    assert(open_list != NULL);
    assert(heuristics.size() > 0);

    for (unsigned int i = 0; i < heuristics.size(); i++)
        heuristics[i]->evaluate(*g_initial_state);
	open_list->evaluate(0, false);
    search_progress.inc_evaluated();

    if(open_list->is_dead_end()) {
        assert(open_list->dead_end_is_reliable());
        cout << "Initial state is a dead end." << endl;
    }
    else {
        search_progress.get_initial_h_values();
        if (f_evaluator) {
            f_evaluator->evaluate(0,false);
            search_progress.report_f_value(f_evaluator->get_value());
        }
        search_progress.check_h_progress();
        SearchNode node = search_space.get_node(*g_initial_state);
        node.open_initial(heuristics[0]->get_value());

        open_list->insert(node.get_state_buffer());
    }
}


void GeneralEagerBestFirstSearch::statistics() const {
    search_progress.print_statistics();

    search_space.statistics();
}

int GeneralEagerBestFirstSearch::step() {
    SearchNode node = fetch_next_node();

    State s = node.get_state();
    if (check_goal_and_set_plan(s))
        return SOLVED;

    vector<const Operator *> applicable_ops;
    g_successor_generator->generate_applicable_ops(s,
            applicable_ops);
    //TODO:CR - implement double evaluation for preferred operators
    for(int i = 0; i < applicable_ops.size(); i++) {
        const Operator *op = applicable_ops[i];
        State succ_state(s, *op);
        search_progress.inc_generated();

        SearchNode succ_node = search_space.get_node(succ_state);

        if(succ_node.is_dead_end()) {
            // Previously encountered dead end. Don't re-evaluate.
            continue;
        } else if(succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.
            for (unsigned int i = 0; i < heuristics.size(); i++) {
                heuristics[i]->reach_state(s, *op, succ_node.get_state());
                heuristics[i]->evaluate(succ_state);
            }
            search_progress.inc_evaluated();
            //TODO:CR - check dead end using the open list
            bool dead_end = false;
            for (unsigned int i = 0; i < heuristics.size(); i++) {
                if(heuristics[i]->is_dead_end()
                   && heuristics[i]->dead_ends_are_reliable()) {
                    dead_end = true;
                    break;
                }
            }
            if (dead_end) {
                succ_node.mark_as_dead_end();
                continue;
            }
            //TODO:CR - add an ID to each state, and then we can use a vector to save per-state information
			int succ_h = heuristics[0]->get_heuristic();
            succ_node.open(succ_h, node, op);
			open_list->evaluate(succ_node.get_g(), false);
            open_list->insert(succ_node.get_state_buffer());

            search_progress.check_h_progress();

        } else if(succ_node.get_g() > node.get_g() + op->get_cost()) {
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
            	//TODO:CR - test if we should add a reevaluate flag and if it helps
                // if we reopen closed nodes, do that
                if(succ_node.is_closed()) {
                    /* TODO: Verify that the heuristic is inconsistent.
                     * Otherwise, this is a bug. This is a serious
                     * assertion because it can show that a heuristic that
                     * was thought to be consistent isn't. Therefore, it
                     * should be present also in release builds, so don't
                     * use a plain assert. */
                	//TODO:CR - add a consistent flag to heuristics, and add an assert here based on it
                    search_progress.inc_reopened();
                }
                succ_node.reopen(node, op);
                heuristics[0]->set_evaluator_value(succ_node.get_h());
				open_list->evaluate(succ_node.get_g(), false);

                open_list->insert(succ_node.get_state_buffer());
            }
            else {
                // if we do not reopen closed nodes, we just update the parent pointers
                succ_node.update_parent(node, op);
            }
        }
    }

    return IN_PROGRESS;
}

SearchNode GeneralEagerBestFirstSearch::fetch_next_node() {
	//TODO:CR - return a pair of SearchNode, bool
    while(true) {
        if(open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;
            assert(false);
            exit(1); // fix segfault in release mode
            // TODO: Deal with this properly. step() should return
            //       failure.
        }
        State state(open_list->remove_min());
        SearchNode node = search_space.get_node(state);

        // If the node is closed, we do not reopen it, as our heuristic
        // is consistent.
        // TODO: check this
        if(!node.is_closed()) {
            node.close();
            assert(!node.is_dead_end());
			update_jump_statistic(node);
            search_progress.inc_expanded();
            return node;
        }
    }
}

void GeneralEagerBestFirstSearch::dump_search_space()
{
  search_space.dump();
}

void GeneralEagerBestFirstSearch::update_jump_statistic(const SearchNode& node) {
	if (f_evaluator) {
		heuristics[0]->set_evaluator_value(node.get_h());
		f_evaluator->evaluate(node.get_g(), false);
		int new_f_value = f_evaluator->get_value();
		search_progress.report_f_value(new_f_value);
	}
}

void GeneralEagerBestFirstSearch::print_heuristic_values(const vector<int>& values) const {
    for(int i = 0; i < values.size(); i++) {
        cout << values[i];
        if(i != values.size() - 1)
            cout << "/";
    }
}

void GeneralEagerBestFirstSearch::set_f_evaluator(ScalarEvaluator* eval) {
    f_evaluator = eval;
}

void GeneralEagerBestFirstSearch::set_open_list(OpenList<state_var_t *> *open) {
    open_list = open;
}
