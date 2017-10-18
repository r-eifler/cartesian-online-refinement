#ifndef CEGAR_COST_SATURATION_H
#define CEGAR_COST_SATURATION_H

#include "split_selector.h"
#include "abstraction.h"

#include <memory>
#include <vector>

namespace utils {
class CountdownTimer;
class RandomNumberGenerator;
}

namespace cegar {
class CartesianHeuristicFunction;
class SubtaskGenerator;

/*
  Get subtasks from SubtaskGenerators, reduce their costs by wrapping
  them in ModifiedOperatorCostsTasks, compute Abstractions, move
  RefinementHierarchies from Abstractions to
  CartesianHeuristicFunctions, allow extracting
  CartesianHeuristicFunctions into AdditiveCartesianHeuristic.
*/
class CostSaturation {
    const std::vector<std::shared_ptr<SubtaskGenerator>> subtask_generators;
    int max_states;
    const int max_non_looping_transitions;
    const double max_time;
    const bool use_general_costs;
	const bool use_all_goals;
    const PickSplit pick_split;
    utils::RandomNumberGenerator &rng;

    std::vector<CartesianHeuristicFunction> heuristic_functions;
    std::vector<int> remaining_costs;
    std::vector<std::vector<int>> remaining_costs_order;
    int num_abstractions;
    int num_states;
    int num_non_looping_transitions;
  
    
    std::vector<Abstraction*> abstractions;
    std::shared_ptr<AbstractTask> abstask;
    std::vector<std::vector<int>> scp_orders;

    void reset(const TaskProxy &task_proxy);
    void reduce_remaining_costs(const std::vector<int> &saturated_costs);
    void reduce_remaining_costs(const std::vector<int> &saturated_costs, int order);
    std::shared_ptr<AbstractTask> get_remaining_costs_task(
        std::shared_ptr<AbstractTask> &parent) const;
    std::shared_ptr<AbstractTask> get_remaining_costs_task(std::shared_ptr<AbstractTask> &parent,  std::vector<int> old_costs, int order);
    std::shared_ptr<AbstractTask> get_remaining_costs_task(std::shared_ptr<AbstractTask> &parent,  std::vector<int> old_costs);
    void add_cost_partitioning(std::vector<int>* cost1, std::vector<int>* cost2);
    bool state_is_dead_end(const State &state) const;
    void build_abstractions(
        const std::vector<std::shared_ptr<AbstractTask>> &subtasks,
        const utils::CountdownTimer &timer,
        std::function<bool()> should_abort);
   
	

public:
    CostSaturation(
        std::vector<std::shared_ptr<SubtaskGenerator>> &subtask_generators,
        int max_states,
        int max_non_looping_transitions,
        double max_time,
        bool use_general_costs,
		bool use_all_goals,
        PickSplit pick_split,
        utils::RandomNumberGenerator &rng);

    std::vector<CartesianHeuristicFunction*> generate_heuristic_functions(
        const std::shared_ptr<AbstractTask> &task);
  
    void print_statistics() const;
	void print_statistics_end() const;
    void recompute_cost_partitioning_unused(int order_id);
    void recompute_cost_partitioning_unused_all();
    void recompute_cost_partitioning(int order_id);
    void recompute_cost_partitioning(std::vector<int> order);    
    void remove_abstraction(int pos);
	std::vector<int> get_original_cost_partitioning(Abstraction* abs);
	void update_h_complete_cost(Abstraction* abs);
	
  	std::vector<std::vector<int>>* get_unused_cost();
    std::vector<int> get_order(int id);
    int number_of_orders();
    void update_order(int id, std::vector<int> new_order);
    bool add_order(std::vector<int> order, int* order_id);
    void delete_order(int order_id);
	
	int compute_threshold();
	const std::shared_ptr<AbstractTask> get_abs_task();
	void reset_abs();
};
}

#endif
