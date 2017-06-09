#include "additive_cartesian_heuristic.h"

#include "cartesian_heuristic_function.h"
#include "cost_saturation.h"
#include "utils.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/logging.h"
#include "../utils/markup.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"

#include <cassert>

using namespace std;

namespace cegar {
static vector<CartesianHeuristicFunction> generate_heuristic_functions(
    const options::Options &opts) {
    g_log << "Initializing additive Cartesian heuristic..." << endl;
    vector<shared_ptr<SubtaskGenerator>> subtask_generators =
        opts.get_list<shared_ptr<SubtaskGenerator>>("subtasks");
    shared_ptr<utils::RandomNumberGenerator> rng =
        utils::parse_rng_from_options(opts);
    CostSaturation cost_saturation(
        subtask_generators,
        opts.get<int>("max_states"),
        opts.get<int>("max_transitions"),
        opts.get<double>("max_time"),
        opts.get<bool>("use_general_costs"),
        static_cast<PickSplit>(opts.get<int>("pick")),
        *rng);
    return cost_saturation.generate_heuristic_functions(
        opts.get<shared_ptr<AbstractTask>>("transform"));
}

AdditiveCartesianHeuristic::AdditiveCartesianHeuristic(
    const options::Options &opts)
    : Heuristic(opts),
      max_states_online(opts.get<int>("max_states_online")),
      max_iter(opts.get<int>("max_iter")),
	  update_h_values(opts.get<int>("update_h_values")),
      heuristic_functions(generate_heuristic_functions(opts)) {
          cout << "Max states online: " << max_states_online << endl;
}

int AdditiveCartesianHeuristic::compute_heuristic(const GlobalState &global_state) {
    State state = convert_global_state(global_state);
    return compute_heuristic(state);
}

int AdditiveCartesianHeuristic::compute_heuristic(const State &state) {
    //cout << "compute_heuristic" << endl;
    int sum_h = 0;
    for (const CartesianHeuristicFunction &function : heuristic_functions) {
        int value = function.get_value(state);
        //cout << value << " ";
        assert(value >= 0);
        if (value == INF)
            return DEAD_END;
        sum_h += value;
    }
    //cout << endl;
    assert(sum_h >= 0);
    return sum_h;
}
    
    
/*
    Return the heuristic value for each heuristic seperately
*/
vector<int> AdditiveCartesianHeuristic::compute_individual_heuristics(const GlobalState &global_state){
    State state = convert_global_state(global_state);
    vector<int> values;
    int sum_h = 0;
    for (const CartesianHeuristicFunction &function : heuristic_functions) {
        int value = function.get_value(state);
        assert(value >= 0);
        if (value == INF)
            return values;
        values.push_back(value);
        sum_h += value;
    }
    assert(sum_h >= 0);
    return values;
}
    
bool AdditiveCartesianHeuristic::online_Refine(const GlobalState &global_state){
   State state = convert_global_state(global_state);
   bool refined = false;
   //refine every heuristic
   //TODO recompute cost partitioning
   for (const CartesianHeuristicFunction &function : heuristic_functions) {
       int refined_states = function.online_Refine(state, max_iter, update_h_values, max_states_online - online_refined_states);
       if(refined_states > 0){
            refined = true;    
       }
       online_refined_states += refined_states;
   }
   return refined;
}
	
void AdditiveCartesianHeuristic::print_statistics(){
		for (const CartesianHeuristicFunction &function : heuristic_functions) {
			function.print_statistics();	
		}
}

static Heuristic *_parse(OptionParser &parser) {
    parser.document_synopsis(
        "Additive CEGAR heuristic",
        "See the paper introducing Counterexample-guided Abstraction "
        "Refinement (CEGAR) for classical planning:" +
        utils::format_paper_reference(
            {"Jendrik Seipp", "Malte Helmert"},
            "Counterexample-guided Cartesian Abstraction Refinement",
            "http://ai.cs.unibas.ch/papers/seipp-helmert-icaps2013.pdf",
            "Proceedings of the 23rd International Conference on Automated "
            "Planning and Scheduling (ICAPS 2013)",
            "347-351",
            "AAAI Press 2013") +
        "and the paper showing how to make the abstractions additive:" +
        utils::format_paper_reference(
            {"Jendrik Seipp", "Malte Helmert"},
            "Diverse and Additive Cartesian Abstraction Heuristics",
            "http://ai.cs.unibas.ch/papers/seipp-helmert-icaps2014.pdf",
            "Proceedings of the 24th International Conference on "
            "Automated Planning and Scheduling (ICAPS 2014)",
            "289-297",
            "AAAI Press 2014"));
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "not supported");
    parser.document_language_support("axioms", "not supported");
    parser.document_property("admissible", "yes");
    // TODO: Is the additive version consistent as well?
    parser.document_property("consistent", "yes");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");

    parser.add_list_option<shared_ptr<SubtaskGenerator>>(
        "subtasks",
        "subtask generators",
        "[landmarks(),goals()]");
    parser.add_option<int>(
        "max_states",
        "maximum sum of abstract states over all abstractions",
        "infinity",
        Bounds("1", "infinity"));
    parser.add_option<int>(
        "max_transitions",
        "maximum sum of real transitions (excluding self-loops) over "
        " all abstractions",
        "1000000",
        Bounds("0", "infinity"));
    parser.add_option<double>(
        "max_time",
        "maximum time in seconds for building abstractions",
        "infinity",
        Bounds("0.0", "infinity"));
    vector<string> pick_strategies;
    pick_strategies.push_back("RANDOM");
    pick_strategies.push_back("MIN_UNWANTED");
    pick_strategies.push_back("MAX_UNWANTED");
    pick_strategies.push_back("MIN_REFINED");
    pick_strategies.push_back("MAX_REFINED");
    pick_strategies.push_back("MIN_HADD");
    pick_strategies.push_back("MAX_HADD");
    parser.add_enum_option(
        "pick", pick_strategies, "split-selection strategy", "MAX_REFINED");
    parser.add_option<bool>(
        "use_general_costs",
        "allow negative costs in cost partitioning",
        "true");
    
    //Online Refinement options
    parser.add_option<int>(
        "max_states_online",
        "maximum sum of abstract states over all abstractions added during online refinement",
        "infinity",
        Bounds("1", "infinity"));
    parser.add_option<int>(
        "max_iter",
        "maximum number of iterations of the refinement algorithm per state",
        "1",
        Bounds("1", "infinity"));
	parser.add_option<int>(
        "update_h_values",
        "number of refined states until the h values of the abstract states are updated",
        "20",
        Bounds("1", "infinity"));

    
    Heuristic::add_options_to_parser(parser);
    utils::add_rng_options(parser);
    Options opts = parser.parse();

    if (parser.dry_run())
        return nullptr;

    return new AdditiveCartesianHeuristic(opts);
}

static Plugin<Heuristic> _plugin("cegar", _parse);
}
