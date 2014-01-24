#include "linear_merge_strategy.h"

#include "../option_parser.h"
#include "../plugin.h"

#include <cassert>
#include <cstdlib>

using namespace std;

LinearMergeStrategy::LinearMergeStrategy(const Options &opts)
    : MergeStrategy(),
      order(LinearMergeStrategyType(opts.get_enum("type"))),
      previous_index(-1) {
}

bool LinearMergeStrategy::done() const {
    return order.done();
}

pair<int, int> LinearMergeStrategy::get_next(const vector<Abstraction *> &all_abstractions) {
    pair<int, int> next_indices;
    if (previous_index == -1) {
        next_indices.first = order.next();
    } else {
        next_indices.first = previous_index;
    }
    next_indices.second = order.next();
    previous_index = next_indices.first;
    assert(all_abstractions[next_indices.first]);
    if (!all_abstractions[next_indices.second]) {
        exit(1);
    }
    assert(all_abstractions[next_indices.second]);
    return next_indices;
}

void LinearMergeStrategy::dump_strategy_specific_options() const {
    order.dump();
}

string LinearMergeStrategy::name() const {
    return "linear";
}

static MergeStrategy *_parse(OptionParser &parser) {
    vector<string> merge_strategies;
    //TODO: it's a bit dangerous that the merge strategies here
    // have to be specified exactly in the same order
    // as in the enum definition. Try to find a way around this,
    // or at least raise an error when the order is wrong.
    merge_strategies.push_back("MERGE_LINEAR_CG_GOAL_LEVEL");
    merge_strategies.push_back("MERGE_LINEAR_CG_GOAL_RANDOM");
    merge_strategies.push_back("MERGE_LINEAR_GOAL_CG_LEVEL");
    merge_strategies.push_back("MERGE_LINEAR_RANDOM");
    merge_strategies.push_back("MERGE_LINEAR_LEVEL");
    merge_strategies.push_back("MERGE_LINEAR_REVERSE_LEVEL");
    parser.add_enum_option("type", merge_strategies,
                           "linear merge strategy",
                           "MERGE_LINEAR_CG_GOAL_LEVEL");

    Options opts = parser.parse();
    if (parser.help_mode())
        return 0;
    if (!parser.dry_run())
        return new LinearMergeStrategy(opts);
    else
        return 0;
}

static Plugin<MergeStrategy> _plugin("merge_linear", _parse);

