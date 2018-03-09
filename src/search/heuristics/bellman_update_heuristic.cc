#include "bellman_update_heuristic.h"

#include "../global_state.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"
#include "../heuristics/ff_heuristic.h"

#include <cstddef>
#include <limits>
#include <utility>

using namespace std;

namespace bellman_update_heuristic {
BellmanUpdateHeuristic::BellmanUpdateHeuristic(const Options &opts)
    : Heuristic(opts),
      min_operator_cost(get_min_operator_cost(task_proxy)) {


		  base_heuristic = new ff_heuristic::FFHeuristic(opts);
    cout << "Initializing blind search heuristic..." << endl;
}


BellmanUpdateHeuristic::~BellmanUpdateHeuristic() {
}

int BellmanUpdateHeuristic::compute_heuristic(const GlobalState &global_state) {
    State state = convert_global_state(global_state);
	if(h_values.find(global_state.get_id()) != h_values.end()){
		return h_values[global_state.get_id()];
	}
	else{
		int h = base_heuristic->compute_heuristic(global_state);
		h_values[global_state.get_id()] = h;
		return h;
	}
}


bool BellmanUpdateHeuristic::online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates){

	int h_s = compute_heuristic(global_state);
	int min_h = 0;
	for(uint i = 0; i < succStates.size(); i++){
		int h_suc = compute_heuristic(succStates[i].first);
		int new_min = h_suc + succStates[i].second;
		if(new_min < min_h){
			min_h = new_min;
		}
	}

	if(min_h > h_s){
		h_values[global_state.get_id()] = min_h;
	}
	else{
		return false;
	}

	return true;
}

static Heuristic *_parse(OptionParser &parser) {
    parser.document_synopsis("Blind heuristic",
                             "Returns cost of cheapest action for "
                             "non-goal states, "
                             "0 for goal states");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "supported");
    parser.document_language_support("axioms", "supported");
    parser.document_property("admissible", "yes");
    parser.document_property("consistent", "yes");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new BellmanUpdateHeuristic(opts);
}

static Plugin<Heuristic> _plugin("bellman_update", _parse);
}
