#ifndef CEGAR_CARTESIAN_HEURISTIC_FUNCTION_H
#define CEGAR_CARTESIAN_HEURISTIC_FUNCTION_H

#include "abstraction.h"
#include "refinement_hierarchy.h"
#include "../heuristics/max_heuristic.h"
#include "../utils/countdown_timer.h"

#include "../task_proxy.h"

#include <memory>

class AbstractTask;
class State;

namespace cegar {
/*
  Store RefinementHierarchy and subtask for looking up heuristic values
  efficiently.
*/
class CartesianHeuristicFunction {
    const std::shared_ptr<AbstractTask> task;
    //TaskProxy task_proxy;
	  Abstraction *abstraction;
  
    const utils::Timer update_timer;
    const int update_counter = 5;
	
	

public:
	int id;
	
	CartesianHeuristicFunction(Abstraction *abs, int id);

    int get_value(const State &parent_state) const;
	int get_original_value(const State &parent_state) const;
	int get_value(const State &parent_state, int order) const;

	//Online Refinement
	std::vector<int> get_values(const State &parent_state) const;
	int online_Refine(const State &state, std::vector<State> goal_states, int max_iter, int max_states_refine, std::vector<std::vector<int>> *unused_cost) const;
	int refineBellmanStyle(const State & state);
	int refineSplitPre(const State & state, const State & prestate);
	int refineSplitPreAction(const State &state, const State &preState, const std::vector<std::pair<int,int>> conditions);
	int refineNewGoal(const State &state, const State &newGoal);
	int refineBasedOnBellman(const State &state, const int bound); 
	void print_statistics() const; 
	void update_h_values();
	void merge(CartesianHeuristicFunction *function);
	void update_h_and_g_values(int pos, bool new_order);
	bool satisfies_goal(State state);
	void set_refined(bool b);
	int get_original_h_value(const State &parent_state) const;
	int size();
	Abstraction* get_abstraction();

};
}

#endif
