#include "bellman_update_heuristic.h"

#include "../global_state.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_tools.h"
#include "../cegar/additive_cartesian_heuristic.h"

#include <cstddef>
#include <limits>
#include <utility>

using namespace std;

namespace bellman_update_heuristic {
BellmanUpdateHeuristic::BellmanUpdateHeuristic(const Options &opts)
    : Heuristic(opts),
      min_operator_cost(get_min_operator_cost(task_proxy)),
	  base_heuristic(opts.get<ScalarEvaluator *>("base", nullptr)),
	  refine_abstractions(opts.get<bool>("refine_abstractions")){


	cout << "Initializing bellman update heuristic heuristic..." << endl;
}


BellmanUpdateHeuristic::~BellmanUpdateHeuristic() {
}

int BellmanUpdateHeuristic::compute_heuristic(const GlobalState &global_state) {
    State state = convert_global_state(global_state);
	if(h_values.find(state.hash()) != h_values.end()){
		int h = h_values[state.hash()];
		if(refine_abstractions){
			Heuristic* heuristic = (Heuristic*) base_heuristic;
			int hCA = heuristic->compute_heuristic(global_state);
			//cout << "h(" << state.hash() << ")=" << h << endl;
			if(h < hCA){
				h_values.erase(h_values.find(state.hash())); //TODO good idea ?
				return hCA;
			}
			return h;
		}
		else{
			return h;
		}
	}
	else{
		Heuristic* heuristic = (Heuristic*) base_heuristic;
		int h = heuristic->compute_heuristic(global_state);
		h_values[state.hash()] = h;
		return h;
	}
}


bool BellmanUpdateHeuristic::online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates, std::vector<GlobalState> newGoals, double time_bound){

	utils::Timer timer;
	timer.resume();

	//cout << "+++++++++++++++ BELLMAN UPDATE +++++++++++++++++" << endl;
    State state = convert_global_state(global_state);
	int h_s = compute_heuristic(global_state);
	//cout << "current h = " << h_s << endl;
	int h_suc = compute_heuristic(succStates[0].first);
	int min_h = h_suc + succStates[0].second;
	for(uint i = 0; i < succStates.size(); i++){
		h_suc = compute_heuristic(succStates[i].first);
		int new_min = h_suc + succStates[i].second;
		//cout << "SUCC h+c=" << new_min << endl;
		if(new_min < min_h){
			min_h = new_min;
		}
	}

	bool updated = false;
	if(min_h > h_s){
		//cout << "INCREASE " << h_s << " -> " << min_h << endl;
		h_values[state.hash()] = min_h;
		updated = true;
	}
	
	if(refine_abstractions){
		double new_bound = time_bound - timer();
		cout << "New time bound: " << new_bound << endl;
		if(new_bound <= 0){
			return updated;
		}

		Heuristic* heuristic = (Heuristic*) base_heuristic;
		return heuristic->online_Refine(global_state, succStates, newGoals, new_bound) || updated;
	}
	else{
		return updated;
	}

}

static Heuristic *_parse(OptionParser &parser) {
    parser.document_synopsis("bellman update heuristic",
                             "TODO");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "supported");
    parser.document_language_support("axioms", "supported");
    parser.document_property("admissible", "yes");
    parser.document_property("consistent", "yes");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");

	parser.add_option<ScalarEvaluator *>("base", "base heuristic");
	parser.add_option<bool>(
        "refine_abstractions",
        "TODO",
        "false");

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new BellmanUpdateHeuristic(opts);
}

static Plugin<Heuristic> _plugin("bellman_update", _parse);
}
