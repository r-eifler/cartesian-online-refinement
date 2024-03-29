#include "additive_cartesian_heuristic.h"

#include "cartesian_heuristic_function.h"
#include "cost_saturation.h"
#include "order_selecter.h"
#include "utils.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/logging.h"
#include "../utils/markup.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"

#include "../evaluation_result.h"


#include <cassert>
#include <algorithm>
#include <chrono>

using namespace std;

namespace cegar {
vector<CartesianHeuristicFunction*> AdditiveCartesianHeuristic::generate_heuristic_functions(
    const options::Options &opts) {
    g_log << "Initializing additive Cartesian heuristic..." << endl;
    vector<shared_ptr<SubtaskGenerator>> subtask_generators =
        opts.get_list<shared_ptr<SubtaskGenerator>>("subtasks");
    shared_ptr<utils::RandomNumberGenerator> rng =
        utils::parse_rng_from_options(opts);
    cost_saturation = new CostSaturation (
        subtask_generators,
        opts.get<int>("max_states"),
        opts.get<int>("max_transitions"),
        opts.get<double>("max_time"),
        opts.get<bool>("use_general_costs"),
		opts.get<bool>("use_all_goals"),
        static_cast<PickSplit>(opts.get<int>("pick")),
        *rng);
    rng_order = &(*rng);
    return cost_saturation->generate_heuristic_functions(
        opts.get<shared_ptr<AbstractTask>>("transform"));
}

AdditiveCartesianHeuristic::AdditiveCartesianHeuristic(
    const options::Options &opts)
    : Heuristic(opts),
      max_states_online(opts.get<int>("max_states_online")),
      max_iter(opts.get<int>("max_iter")),
	  update_h_values(opts.get<int>("update_h_values")),
	  use_all_goals(opts.get<bool>("use_all_goals")),
	  use_merge(opts.get<bool>("use_merge")),
	  prove_bellman(opts.get<bool>("prove_bellman")),
	  threshold(opts.get<int>("threshold")),
	  local_minimum(opts.get<bool>("local_minimum")),
	  strategy(static_cast<Strategy>(opts.get<int>("strategy"))),
      heuristic_functions(generate_heuristic_functions(opts)),
      onlineRefinement(cost_saturation, rng_order, max_states_online, opts.get<bool>("use_useful_split")),
      merge(cost_saturation, rng_order, static_cast<MergeStrategy>(opts.get<int>("merge_strategy"))){
		  cout << "Max states online: " << max_states_online << endl;
		  refine_timer.reset();
          refine_timer.stop();
		  cost_timer.reset();
          cost_timer.stop();
		  update_timer.reset();
          update_timer.stop();
		  prove_timer.reset();
          prove_timer.stop();
		  merge_timer.reset();
          merge_timer.stop();
		  values_timer.reset();
          values_timer.stop();
		
		  
          orderSelecter = new OrderSelecter(static_cast<CostOrder>(opts.get<int>("order")), cost_saturation, rng_order);
          orderSelecter->set_heuristic_functions(&heuristic_functions);
          onlineRefinement.set_heuristic_functions(&heuristic_functions);
          merge.set_heuristic_functions(&heuristic_functions);
          
          
     usefullnes_of_order.push_back(0);
     lifetime_of_order.push_back(0);
     for(size_t i = 0; i < heuristic_functions.size(); i++){
         usefullnes_of_abstraction.push_back(0);
     }
     
	//init threshold 
	if(threshold == -1){
		threshold = cost_saturation->compute_threshold();
		cout << "Threshold: " << threshold << endl;
	}
		  
}

int AdditiveCartesianHeuristic::compute_heuristic(const GlobalState &global_state) {
    State state = convert_global_state(global_state);
    return compute_heuristic(state);
}
    
    
int AdditiveCartesianHeuristic::compute_current_order_heuristic(State state){
    //cout << "Current Order values: ";
    int sum_h = 0;
    for (const CartesianHeuristicFunction *function : heuristic_functions) {
        
        int value = function->get_value(state);
        //cout << value << " ";
        assert(value >= 0);
        if (value == INF)
            return DEAD_END;
        sum_h += value;
    }
    //Fcout << endl;
    assert(sum_h >= 0);
    return sum_h;
}

int AdditiveCartesianHeuristic::compute_heuristic(const State &state) {
	
    //cout << "-------------- compute_heuristic ---------------" << endl;
    update_timer.resume();
    vector<int> sums_h(number_of_orders, 0);
    int nf = 0;
    for (const CartesianHeuristicFunction *function : heuristic_functions) {
        bool use = false;
        values_timer.resume();
        std::vector<int> values = function->get_values(state);
        values_timer.stop();
        for(size_t i = 0; i < values.size(); i++){
            if(values[i] > 0){
                use = true;   
            }
            //cout << values[i] << " ";
            if (values[i] == INF){
                return DEAD_END;
			}
            int sum = sums_h[i] + values[i]; 
            
            
            sums_h[i] = sum;
        }
        //cout << endl;
        if(use)
            usefullnes_of_abstraction[nf++]++;
    }
    //cout << "-----------------------------" << endl;
    //compute MAX
    int max = sums_h[0];
    int pos_max = 0;
    for(size_t i = 0; i < sums_h.size(); i++){
        //cout << sums_h[i] << " ";
        lifetime_of_order[i]++;
        if(sums_h[i] > max){
            max = sums_h[i];
            pos_max = i;
        } 
    }
    /*
    cout << endl;
    cout << "---> " << max << " order: " << pos_max << endl;
    cout << "-----------------------------" << endl;
    */
    /*
    for(size_t i = 0; i < sums_h.size(); i++){
        if(sums_h[i] == max)
            usefullnes_of_order[i]++;
    }
    */
    current_order = pos_max;
    usefullnes_of_order[pos_max]++;
    update_timer.stop();
    return max;
}
	
std::vector<int> AdditiveCartesianHeuristic::compute_individual_heuristics_of_order(const GlobalState &global_state, int order){
	State state = convert_global_state(global_state);
	return compute_individual_heuristics_of_order(state, order);
}
    
std::vector<int> AdditiveCartesianHeuristic::compute_individual_heuristics_of_order(const State state, int order){
    vector<int> values;
    for (const CartesianHeuristicFunction *function : heuristic_functions) {
        int value = function->get_value(state, order);
		//cout << "value: " << value << " order: " << order << endl;
        assert(value >= 0);
        //if (value == INF)
        //    return values;
        values.push_back(value);
    }
    return values;
}
	
std::vector<int> AdditiveCartesianHeuristic::compute_original_individual_heuristics(State state){	
	vector<int> values;
    int sum_h = 0;
    for (const CartesianHeuristicFunction *function : heuristic_functions) {
        int value = function->get_original_value(state);
        assert(value >= 0);
        values.push_back(value);
        sum_h += value;
    }
    assert(sum_h >= 0);
    return values;
}
    
    
/*
    Return the heuristic value for each heuristic seperately
*/
vector<int> AdditiveCartesianHeuristic::compute_individual_heuristics(const GlobalState &global_state){
    State state = convert_global_state(global_state);
    vector<int> values;
    int sum_h = 0;
    for (const CartesianHeuristicFunction *function : heuristic_functions) {
        int value = function->get_value(state);
        assert(value >= 0);
        if (value == INF)
            return values;
        values.push_back(value);
        sum_h += value;
    }
    assert(sum_h >= 0);
    return values;
}
    
void AdditiveCartesianHeuristic::print_order(){
    for (const CartesianHeuristicFunction *function : heuristic_functions) {
        cout << function->id << " ";
    }
    cout << endl;
}
    
bool AdditiveCartesianHeuristic::online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates){
   //cout << "--------------------------------------------------------------------------------" << endl;
       
    State state = convert_global_state(global_state);
	
    
	bool conflict = false;
	int h_value = 0;
	vector<bool> toRefine;
		
	bool bellman = prove_bellman_individual(global_state, succStates, &toRefine, &h_value, &conflict);
	if(bellman){
		sat_bellman++;
		return false;	
	}
	not_sat_bellman++;
	
	
    //Refinement pathology   
    if( use_merge && heuristic_functions.size() > 1 && conflict && !use_all_goals ){
		merge_timer.resume();
        refinement_pathology++;
		//modify refined -> otherwise no merge is selected 
		vector<bool> modified_toRefine(toRefine.size(), true);
        int merged = merge.merge(modified_toRefine);
      
       //Check if merge improved the heuristic
       if(merged){
		   int new_h_value = compute_heuristic(state);
		   if(false && h_value <  new_h_value){   
				cout << "Merge Heuristic has been improved h_old(s) = " << h_value << " --> h_new(s) = " << new_h_value << endl;
		   }
		   assert(new_h_value >= h_value); 
       }
		merge_timer.stop();
       return true; 
    }
	
	switch (strategy){
		case Strategy::ORDER_REFINE : 
			if(reorder(state, &h_value, toRefine)){
				return true;	
			}
			return refine(state, &h_value, toRefine);
		case Strategy::REFINE_ORDER : 
			if(refine(state, &h_value, toRefine)){
				return true;	
			}
			return reorder(state, &h_value, toRefine);
		case Strategy::ONLY_REFINE:
			return refine(state, &h_value, toRefine);
		case Strategy::ONLY_ORDER:
			return reorder(state, &h_value, toRefine);			
	}
	
   //cout << "--------------------------------------------------------------------------------" << endl;
   return true;  
}
	
bool AdditiveCartesianHeuristic::prove_bellman_sum(GlobalState global_state, std::vector<std::pair<GlobalState, int>> succStates, int* current_h){
	prove_timer.resume();

	int infinity = EvaluationResult::INFTY;
	//heuristc value of currently expanded state
	*current_h = compute_heuristic(global_state);
	//Compute provable h values for all successor states   
	int provable_h_value = infinity;

	for (pair<GlobalState, int> succ : succStates) {
		int succ_h_value = compute_heuristic(succ.first);
		provable_h_value = min(
				provable_h_value,
				(succ_h_value == infinity) ? infinity : succ_h_value + succ.second);
		
		if(provable_h_value <= *current_h){
			prove_timer.stop();
			//cout << "BEllMAN OK" << endl;
			return true;
		}
	}
	prove_timer.stop();
	//cout << "BEllMAN FAIL" << endl;
	return false;
}
	
bool AdditiveCartesianHeuristic::prove_bellman_individual(GlobalState global_state, vector<pair<GlobalState, int>> succStates, vector<bool> *toRefine, int* current_h, bool* conflict){
	prove_timer.resume();
	//cout << "---------------------------------------" << endl;
	bool debug = false;
	*current_h = 0;
	toRefine->clear();
	int infinity = EvaluationResult::INFTY;
	//heuristc value of currently expanded state
	vector<int> h_values = compute_individual_heuristics_of_order(global_state, current_order);
	if(debug)
		cout << "h value:       ";
	for(int v : h_values){
		if(debug)
			cout << v << " ";
		*current_h += v;   
	}
	if(debug){		
		cout << " = " << *current_h << endl;
	}
	
	

	//Compute provable h values for all successor states
	vector<int> provable_h_values;   
	int provable_h_value = infinity;
	//init
	for(uint i = 0; i < h_values.size(); i++){
		   provable_h_values.push_back(infinity);
	}
	for (pair<GlobalState, int> succ : succStates) {
		string succ_h_values("succ h values: ");
		vector<int> succ_values = compute_individual_heuristics_of_order(succ.first, current_order);
		assert(succ_values.size() == heuristic_functions.size());
		int succ_h_value = 0;
		for(int v : succ_values){
			succ_h_value += v;   
		}
		if(local_minimum && succ_h_value <= *current_h){
			prove_timer.stop();
			//cout << "NO LOCAL MINIMUM" << endl;
			return true;
		}
		provable_h_value = min(
				provable_h_value,
				(succ_h_value == infinity) ? infinity : succ_h_value + succ.second);
		
		if(provable_h_value <= *current_h + threshold){
			prove_timer.stop();
			return true;	
		}
		
		for(uint i = 0; i < provable_h_values.size(); i++){
			succ_h_values += to_string(succ_values[i]) + " ";
			provable_h_values[i] = min(
				provable_h_values[i],
				(succ_values[i] == infinity) ? infinity : succ_values[i] + succ.second);
		}
		
		if(debug)
			cout << succ_h_values <<  " = " << succ_h_value << endl;
	}
	//cout << "LOCAL MINIMUM" << endl;
	//Check if sum could be refined
	bool refine_sum = provable_h_value > *current_h + threshold;
	//cout << provable_h_value << " > " << *current_h << " +  " << threshold << endl;

	if(!refine_sum){
		if(debug)
			cout << "---> not improvable" << endl;       
		prove_timer.stop();
		//cout << "	BELLMAN TRUE" << endl;
		return true;    
	}

	

	if(debug)
		cout << "---> h(s) = " << *current_h << " improvable to " << provable_h_value << endl;

	
	
	//Check which heuristic could be refined
	string provable_h_values_s("provable h values: ");
	for(uint i = 0; i < provable_h_values.size(); i++){
		provable_h_values_s += to_string(provable_h_values[i]) + " ";
		if(provable_h_values[i] > h_values[i]){
			*conflict = false;
			//toRefine->push_back(true);
			provable_h_values_s += "r ";   
		}
		else{
			//toRefine->push_back(false);
			provable_h_values_s += "f ";
		}
	}
	if(debug){
		cout << provable_h_values_s << endl;
		cout << "Refinment pathology: " << *conflict << endl;
	}
	prove_timer.stop();
	//cout << "	BELLMAN FALSE" << endl;
	
	//In the canonical version all abstractions are refined
	for(size_t i = 0; i < heuristic_functions.size(); i++){
			toRefine->push_back(true);	
	}
	return false;
}
	
bool AdditiveCartesianHeuristic::refine(State state, int* current_max_h, std::vector<bool> &toRefine){
	bool still_refinable = true;
	int refinement_steps = 0;
	refined_states_total++;
	while(refinement_steps < max_iter){
		//cout << "	Refinement steps: " << refinement_steps << " still refinable: " << still_refinable << endl;
		//if not refinable merge 
		if(!still_refinable){
			//----------- MERGE -------------
			merge_timer.resume();
			if(!use_merge || heuristic_functions.size() == 1 || !merge.merge(toRefine)){
				merge_timer.stop();
				break;	
			}
			still_refinable = true;
			toRefine.clear();
			for(size_t i = 0; i < heuristic_functions.size(); i++){
				toRefine.push_back(true);	
			}
			merge_timer.stop();
			//------------------------
			continue;
		}				
		//------------- REFINE -----------------
		refine_timer.resume();
		still_refinable = onlineRefinement.refine(state, toRefine);

		if(still_refinable){
		   int new_h_value = compute_heuristic(state);
		   if(*current_max_h <  new_h_value){

			   improved_refine++;    
			   if(false){
					cout << "Refine Heuristic with order " << current_order << " has been improved h_old(s) = " << *current_max_h << " --> h_new(s) = " << new_h_value << endl;
			   }
			   *current_max_h = new_h_value;
			   refine_timer.stop();
			   return true;
		   }
		}
		refine_timer.stop();
		
		//------------------------------
		refinement_steps++;
		refine_steps_total++;		
	}

	return false;
}

bool AdditiveCartesianHeuristic::reorder(State state, int* current_max_h, std::vector<bool> &toRefine){
	if(heuristic_functions.size() > 1){
		cost_timer.resume();
		//get current maximal order
		vector<int> updated_order(cost_saturation->get_order(current_order));
		/*
		cout << "Order: ";
		for(int v : updated_order){
			cout << v << " ";	
		}
		cout << endl;
		*/
		//compute new order and update the cost partitioning
		vector<int> new_order = orderSelecter->compute(state, current_order, updated_order, *current_max_h, toRefine, this);
		cost_saturation->recompute_cost_partitioning(new_order);

	   //Check if heuristic has been improved
	   int new_h_value = compute_current_order_heuristic(state);
		//cout << "Order " << current_order << " Heuristic h_old(s) = " << *current_max_h << " --> h_new(s) = " << new_h_value << endl;
	   if(*current_max_h <  new_h_value){ 
		   int new_scp_order = current_order;
		   //check if order already exists
				//TODO: There are states where an existing order whith completely new computed cost partitioning 
				//has a higher value than the stepwise addition of the unused cost
				//--> should the same order be added multiple times ?

		   //Add the new order to the cost partitioning
		   bool exists = cost_saturation->add_order(new_order, &new_scp_order);
		   if(!exists){
			   	current_order = new_scp_order;
				if(false){
					cout << "Order " << current_order << " Heuristic has been improved h_old(s) = " << *current_max_h << " --> h_new(s) = " << new_h_value << endl; 
				}
				improved_order++;
				number_of_orders++;
			    *current_max_h = new_h_value;
				usefullnes_of_order.push_back(0);
				lifetime_of_order.push_back(0);
				//update the h values of the abstractions
				for (CartesianHeuristicFunction *function : heuristic_functions) {
					function->update_h_and_g_values(new_scp_order, true);
				}			    
				cost_timer.stop();
				return true;
		   }
		}       
		cost_timer.stop();
	}
	return false;
}
    	
void AdditiveCartesianHeuristic::print_statistics(){
        cout << "Order: ";
        print_order();
		for (const CartesianHeuristicFunction *function : heuristic_functions) {
            cout << "-------------- Heuristic: " << function->id << "--------------------" << endl; 
			function->print_statistics();	
		}
        cout << endl;
        cout << "Total refined states: " << (sat_bellman + not_sat_bellman) << endl;              
        cout << "Refinement Pathology: " << refinement_pathology << endl;
    
        cout << endl << "Times: " << endl;
        cout << "Heuristic evaluation time: " << update_timer << endl;
        cout << "Get values time: " << values_timer << endl;
    	cout << endl;
	
		cout << "---------------- BELLMAN ----------------- " << endl;
		cout << "Bellman proof time: " << prove_timer << endl;
		cout << "States satisfied bellman equation: " << sat_bellman << endl;
		cout << "States not satisfied bellman equation: " << not_sat_bellman << endl;
		cout << endl;
	
		orderSelecter->print_statistics();
		cout << "Reorder time: " << cost_timer << endl;
		cout << "Improved States order: " << improved_order << endl;
		cout << endl;
	
		onlineRefinement.print_statistics();
		cout << "Refine time: " << refine_timer << endl;
		cout << "Improved States refine: " << improved_refine << endl;
		cout << "Average number of refine steps per state: " << (refine_steps_total / (double) refined_states_total) << endl;
		cout << endl;
	
        merge.print_statistics();
		cout << "Merge time: " << merge_timer << endl;
		cout << "Improved States merge: " << improved_merge << endl;	
		cout << endl;
    
        cout << endl;
		cout << "---------------- USEFULNES ----------------- " << endl;
        cout << "Usefulnes of order: ";
        for(int n : usefullnes_of_order){
            cout << n << " ";   
        }
        cout << endl;
        cout << "Lifetime of order: ";
        for(int n : lifetime_of_order){
            cout << n << " ";   
        }
        cout << endl;
		cout << "Ratio: ";
        for(uint i = 0; i < usefullnes_of_order.size(); i++){
            cout << (float)usefullnes_of_order[i] / lifetime_of_order[i] << " ";    
        }
        cout << endl;
    	cout << endl;
		cout << "----- ABSTRACTIONS -----" << endl;
        cout << "usefullnes: " ;
        for(int u : usefullnes_of_abstraction){
            cout << u << " ";   
        }
		cout << endl;
    	
        cout << endl;
        cost_saturation->print_statistics_end();
		
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
        "infinity",
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
        "false");
    
    //Online Refinement options
    parser.add_option<int>(
        "max_states_online",
        "maximum sum of abstract states over all abstractions added during online refinement",
        "infinity",
        Bounds("1", "infinity"));
    parser.add_option<int>(
        "max_iter",
        "maximum number of iterations of the refinement algorithm per state",
        "infinity",
        Bounds("1", "infinity"));
	parser.add_option<int>(
        "update_h_values",
        "number of refined states until the h values of the abstract states are updated",
        "20",
        Bounds("1", "infinity"));
	parser.add_option<bool>(
        "use_all_goals",
        "only use the diversification on goal facts during the offline phase",
        "false");
	parser.add_option<bool>(
        "use_merge",
        "TODO",
        "true");
	parser.add_option<bool>(
        "use_useful_split",
        "split the states in the online refinement step based on the unused cost",
        "false");
	parser.add_option<bool>(
        "prove_bellman",
        "TODO",
        "true");
	parser.add_option<int>(
        "threshold",
        "threshold which the local error has to exceed to be refined",
        "0",
        Bounds("-1", "100"));
	parser.add_option<bool>(
        "local_minimum",
        "test of local minimum instead of bellman",
        "false");
	
	vector<string> refine_strategies;
	refine_strategies.push_back("ORDER_REFINE");
	refine_strategies.push_back("REFINE_ORDER");
	refine_strategies.push_back("ONLY_ORDER");
	refine_strategies.push_back("ONLY_REFINE");
	parser.add_enum_option(
        "strategy", refine_strategies, "order of reorder abd refine strategy", "ORDER_REFINE");
	
	vector<string> merge_strategies;
	merge_strategies.push_back("COMPATIBLE_PLANS");
	merge_strategies.push_back("SMALLEST");
	parser.add_enum_option(
        "merge_strategy", merge_strategies, "selection strategy to decide which abstractions are merged", "SMALLEST");
		
	//Different Order selection strategies
	vector<string> order_strategies;
    order_strategies.push_back("SHUFFLE");
    order_strategies.push_back("SWAP_FIRST_SAT");
    order_strategies.push_back("BUBBLE");
    order_strategies.push_back("SWAP_ALL_CHECK");
    order_strategies.push_back("SHUFFLE_CHECK");
    order_strategies.push_back("ORDER_ASC");
    order_strategies.push_back("ORDER_DESC");
	order_strategies.push_back("ORDER_ORG_ASC");
    order_strategies.push_back("ORDER_ORG_DESC");
	order_strategies.push_back("HILL_CLIMB");
    parser.add_enum_option(
        "order", order_strategies, "scp order strategy", "ORDER_ORG_DESC");
	

    
    Heuristic::add_options_to_parser(parser);
    utils::add_rng_options(parser);
    Options opts = parser.parse();

    if (parser.dry_run())
        return nullptr;

    return new AdditiveCartesianHeuristic(opts);
}

static Plugin<Heuristic> _plugin("cegar", _parse);
}
