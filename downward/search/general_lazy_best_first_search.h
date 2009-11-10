#ifndef GENERALLAZYBESTFIRSTSEARCH_H_
#define GENERALLAZYBESTFIRSTSEARCH_H_

#include <vector>

#include "closed_list.h"
#include "open-lists/open_list.h"
#include "search_engine.h"
#include "state.h"
#include "scalar_evaluator.h"
#include "search_space.h"

class Heuristic;
class Operator;


typedef pair<state_var_t *, const Operator *> OpenListEntryLazy;

class GeneralLazyBestFirstSearch: public SearchEngine {
protected:
    SearchSpace search_space;
    OpenList<OpenListEntryLazy> *open_list;
    ScalarEvaluator *f_evaluator;
    vector<Heuristic *> heuristics;
    vector<Heuristic *> preferred_operator_heuristics;

    vector<int> best_heuristic_values;
    int generated_states;

    State current_state;
    state_var_t *current_predecessor_buffer;
    const Operator *current_operator;
    int current_g;

    virtual void initialize();
    virtual int step();

    void generate_successors();
    int fetch_next_state();

    bool check_goal();
    bool check_progress();
    void report_progress();
    void reward_progress();

    void set_open_list(OpenList<OpenListEntryLazy> *open);
public:
    GeneralLazyBestFirstSearch();
    virtual ~GeneralLazyBestFirstSearch();

    virtual void add_heuristic(Heuristic *heuristic, bool use_estimates,
                                   bool use_preferred_operators);

    virtual void statistics() const;
};

#endif /* GENERALLAZYBESTFIRSTSEARCH_H_ */
