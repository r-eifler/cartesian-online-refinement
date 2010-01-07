#include "enforced_hill_climbing_search.h"
#include "successor_generator.h"
#include "heuristic.h"
#include "operator.h"
#include "open_lists/standard_scalar_open_list.h"
#include "open_lists/tiebreaking_open_list.h"
#include "pref_evaluator.h"

EnforcedHillClimbingSearch::EnforcedHillClimbingSearch():
    current_state(*g_initial_state)
{
    heuristic = NULL;
    use_preferred = false;
    preferred_usage = prune_by_preferred;

    expanded = 0;
    generated_states = 0;
    generated_ops = 0;
    num_ehc_phases = 0;

    g_evaluator = new GEvaluator();
}

EnforcedHillClimbingSearch::~EnforcedHillClimbingSearch() {
    delete g_evaluator;
}

void EnforcedHillClimbingSearch::evaluate(const State &state) {
    if (!preferred_contains_eval) {
         heuristic->evaluate(state);
     }
     for (int i = 0; i < preferred_heuristics.size(); i++) {
         preferred_heuristics[i]->evaluate(state);
     }
}

void EnforcedHillClimbingSearch::initialize() {
    assert(heuristic != NULL);

    cout << "Conducting Enforced Hill Climbing Search" << endl;
    if (use_preferred) {
        cout << "Using only preferred operators" << endl;
    }

    evaluate(current_state);
    if (heuristic->is_dead_end()) {
        cout << "Initial state is a dead end, no solution" << endl;
        return;
    }
    current_h = heuristic->get_heuristic();
    search_space.get_node(current_state).open_initial(current_h);

    if (!use_preferred || (preferred_usage == prune_by_preferred)) {
        open_list = new StandardScalarOpenList<OpenListEntryEHC>(g_evaluator, false);
    }
    else {
        vector<ScalarEvaluator *> evals;
        evals.push_back(g_evaluator);
        evals.push_back(new PrefEvaluator);
        open_list = new TieBreakingOpenList<OpenListEntryEHC>(evals, false);
    }
}

void EnforcedHillClimbingSearch::get_successors(const State &state, vector<const Operator *> &ops) {
    if (!use_preferred || preferred_usage == rank_preferred_first) {
        g_successor_generator->generate_applicable_ops(state, ops);

        // mark preferred operators as preferred
        if (use_preferred && (preferred_usage == rank_preferred_first)) {
            for (int i = 0; i < ops.size(); i++) {
                ops[i]->unmark();
            }
            vector<const Operator *> preferred_ops;
            for (int i = 0; i < preferred_heuristics.size(); i++) {
                preferred_heuristics[i]->get_preferred_operators(preferred_ops);
            }
            for (int i = 0; i < preferred_ops.size(); i++) {
                preferred_ops[i]->mark();
            }
        }

    }
    else {
        vector<const Operator *> preferred_ops;
        for (int i = 0; i < preferred_heuristics.size(); i++) {
            preferred_heuristics[i]->get_preferred_operators(preferred_ops);
            for (int j = 0; j < preferred_ops.size(); j++) {
                if (!preferred_ops[j]->is_marked()) {
                    preferred_ops[j]->mark();
                    ops.push_back(preferred_ops[j]);
                }
            }
        }
    }
    expanded++;
    generated_ops += ops.size();
    random_shuffle( ops.begin(), ops.end() );
}

int EnforcedHillClimbingSearch::step() {
    //cout << "s = ";
    //for (int i = 0; i < g_variable_domain.size(); i++) {
    //    cout << current_state[i] << " ";
    //}
    //cout << endl;
    last_expanded = expanded;
    cout << "h = " << current_h << " [expanded " << expanded << " states(s)]" << endl;

    // current_state is the current state, and it is the last state to be evaluated
    // cuurent_h is the h value of the current state

    if ((current_h == 0) && test_goal(current_state)) {
        cout << "Solution found" << endl;
        Plan plan;
        search_space.trace_path(current_state, plan);
        set_plan(plan);
        return SOLVED;
    }

    vector<const Operator *> ops;
    get_successors(current_state, ops);

    SearchNode current_node = search_space.get_node(current_state);
    current_node.close();

    for (int i = 0; i < ops.size(); i++) {
        int d = ops[i]->get_cost();
        OpenListEntryEHC entry = make_pair(current_node.get_state(), make_pair(d, ops[i]));
        open_list->evaluate(d, ops[i]->is_marked());
        open_list->insert(entry);
        ops[i]->unmark();
    }
    return ehc();
}

int EnforcedHillClimbingSearch::ehc() {
    while (!open_list->empty()) {
        OpenListEntryEHC next = open_list->remove_min();
        State &last_parent = next.first;
        int d = next.second.first;
        const Operator *last_op = next.second.second;
        State s(last_parent, *last_op);
        generated_states++;

        SearchNode node = search_space.get_node(s);

        if (node.is_new()) {
            evaluate(s);

            if (heuristic->is_dead_end()) {
                node.mark_as_dead_end();
                continue;
            }

            int h = heuristic->get_heuristic();
            node.open(h, search_space.get_node(last_parent), last_op);

            if (h < current_h) {

                num_ehc_phases++;
                if (d_counts.find(d) == d_counts.end()) {
                    d_counts[d] = make_pair(0, 0);
                }
                pair<int, int> p = d_counts[d];
                p.first = p.first + 1;
                p.second = p.second + expanded - last_expanded;
                d_counts[d] = p;

                current_state = node.get_state();
                current_h = heuristic->get_heuristic();
                open_list->clear();
                return IN_PROGRESS;
            }
            else {
                vector<const Operator *> ops;
                get_successors(s, ops);

                node.close();
                for (int i = 0; i < ops.size(); i++) {
                    int new_d = d + ops[i]->get_cost();
                    OpenListEntryEHC entry = make_pair(s, make_pair(new_d, ops[i]));
                    open_list->evaluate(new_d, ops[i]->is_marked());
                    open_list->insert(entry);
                    ops[i]->unmark();
                }
            }
        }
        //else if ((search_space.get_node(last_parent).get_g() + last_op->get_cost()) < node.get_g()) {
        //    node.reopen(search_space.get_node(last_parent), last_op);
        //}

    }
    cout << "No solution - FAILED" << endl;
    return FAILED;
}

void EnforcedHillClimbingSearch::statistics() const {
    cout << "Expanded: " << expanded << " state(s)" << endl;
    cout << "Generated: " << generated_states  << " state(s)" << endl;
    cout << "Generated Ops: " << generated_ops  << endl;
    cout << "EHC Phases: " << num_ehc_phases << endl;
    cout << "Average expansions per EHC Phase: " << (double) expanded / (double)num_ehc_phases << endl;

    map<int, pair<int, int> >::const_iterator it;
    for (it = d_counts.begin(); it != d_counts.end(); it++) {
        pair<int, pair<int, int> > p = *it;
        cout << "EHC phases of depth "<< p.first << " : " << p.second.first << " - Avg. Expansions: " << (double) p.second.second / (double) p.second.first << endl;
    }

}

void EnforcedHillClimbingSearch::add_heuristic(Heuristic *h, bool use_estimates,
                           bool use_preferred_operators) {
    if (use_estimates) {
        if (heuristic != NULL) {
            cerr << "Only one heuristic for enforced hill climbing. Ignoring last." << endl;
        }
        heuristic = h;
        preferred_contains_eval = use_preferred_operators;
    }

    if (use_preferred_operators) {
        use_preferred = true;
        preferred_heuristics.push_back(h);
    }
}
