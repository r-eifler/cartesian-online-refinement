#include "heuristic.h"

#include "evaluation_context.h"
#include "evaluation_result.h"
#include "global_operator.h"
#include "globals.h"
#include "option_parser.h"
#include "plugin.h"

#include "tasks/cost_adapted_task.h"

#include <cassert>
#include <cstdlib>
#include <limits>

using namespace std;

Heuristic::Heuristic(const Options &opts)
    : description(opts.get_unparsed_config()),
      heuristic_cache(HEntry(NO_VALUE, true)), //TODO: is true really a good idea here?
      //cache_h_values(opts.get<bool>("cache_estimates")),
	  cache_h_values(false),
      task(opts.get<shared_ptr<AbstractTask>>("transform")),
      task_proxy(*task) {
}

Heuristic::~Heuristic() {
}

void Heuristic::set_preferred(const GlobalOperator *op) {
    preferred_operators.insert(op);
}

void Heuristic::set_preferred(const OperatorProxy &op) {
    set_preferred(op.get_global_operator());
}

bool Heuristic::online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates){
	cout << "online refine not implemented " << global_state.get_id()  << "#succ States: " << succStates.size() << endl;	
	return false;
}

void Heuristic::print_statistics(){
		cout << "not implemented " << endl;
}

std::vector<int> Heuristic::compute_individual_heuristics(const GlobalState &global_state){
	cout << "not implemented " << global_state.get_id()  << endl;
	vector<int> v;
	return v;
}

void Heuristic::change_to_order(int id){
	cout << "Change to order: " << id ;
}

bool Heuristic::notify_state_transition(
    const GlobalState & /*parent_state*/,
    const GlobalOperator & /*op*/,
    const GlobalState & /*state*/) {
    return false;
}

State Heuristic::convert_global_state(const GlobalState &global_state) const {
    State state(*g_root_task(), global_state.get_values());
    return task_proxy.convert_ancestor_state(state);
}

void Heuristic::add_options_to_parser(OptionParser &parser) {
    parser.add_option<shared_ptr<AbstractTask>>(
        "transform",
        "Optional task transformation for the heuristic."
        " Currently, adapt_costs() and no_transform() are available.",
        "no_transform()");
    parser.add_option<bool>("cache_estimates", "cache heuristic estimates", "true");
}

// This solution to get default values seems nonoptimal.
// This is currently only used by the LAMA/FF synergy.
Options Heuristic::default_options() {
    Options opts = Options();
    opts.set<shared_ptr<AbstractTask>>("transform", g_root_task());
    opts.set<bool>("cache_estimates", false);
    return opts;
}

EvaluationResult Heuristic::compute_result(EvaluationContext &eval_context) {
	//cout << "Heuristic::compute_result" << endl;
    EvaluationResult result;

    assert(preferred_operators.empty());

    const GlobalState &state = eval_context.get_state();
    bool calculate_preferred = eval_context.get_calculate_preferred();

    int heuristic = NO_VALUE;

    if (!calculate_preferred && cache_h_values &&
        heuristic_cache[state].h != NO_VALUE && !heuristic_cache[state].dirty) {
        heuristic = heuristic_cache[state].h;
        result.set_count_evaluation(false);
		cout << "use cached values" << endl;
    } else {
        heuristic = compute_heuristic(state);
        if (cache_h_values) {
            heuristic_cache[state] = HEntry(heuristic, false);
        }
        result.set_count_evaluation(true);
    }

	//TODO why
	if(!(heuristic == DEAD_END || heuristic >= 0)){
		cout << "Heuristic value not valid: " << heuristic << endl;	
	}
    assert(heuristic == DEAD_END || heuristic >= 0);

    if (heuristic == DEAD_END) {
        /*
          It is permissible to mark preferred operators for dead-end
          states (thus allowing a heuristic to mark them on-the-fly
          before knowing the final result), but if it turns out we
          have a dead end, we don't want to actually report any
          preferred operators.
        */
        preferred_operators.clear();
        heuristic = EvaluationResult::INFTY;
    }

#ifndef NDEBUG
    if (heuristic != EvaluationResult::INFTY) {
        for (const GlobalOperator *op : preferred_operators)
            assert(op->is_applicable(state));
    }
#endif

    result.set_h_value(heuristic);
    result.set_preferred_operators(preferred_operators.pop_as_vector());
    assert(preferred_operators.empty());

    return result;
}

string Heuristic::get_description() const {
    return description;
}


static PluginTypePlugin<Heuristic> _type_plugin(
    "Heuristic",
    "A heuristic specification is either a newly created heuristic "
    "instance or a heuristic that has been defined previously. "
    "This page describes how one can specify a new heuristic instance. "
    "For re-using heuristics, see OptionSyntax#Heuristic_Predefinitions.\n\n"
    "Definitions of //properties// in the descriptions below:\n\n"
    " * **admissible:** h(s) <= h*(s) for all states s\n"
    " * **consistent:** h(s) <= c(s, s') + h(s') for all states s "
    "connected to states s' by an action with cost c(s, s')\n"
    " * **safe:** h(s) = infinity is only true for states "
    "with h*(s) = infinity\n"
    " * **preferred operators:** this heuristic identifies "
    "preferred operators ");
