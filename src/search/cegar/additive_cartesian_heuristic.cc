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
      heuristic_functions(generate_heuristic_functions(opts)) {
          cout << "Max states online: " << max_states_online << endl;
          refine_timer.stop();
          cost_timer.stop();
          update_timer.stop();
          prove_timer.stop();
          merge_timer.stop();
          values_timer.stop();
          
     usefullnes_of_order.push_back(0);
     lifetime_of_order.push_back(0);
     for(size_t i = 0; i < heuristic_functions.size(); i++){
         usefullnes_of_abstraction.push_back(0);
     }
     
}

int AdditiveCartesianHeuristic::compute_heuristic(const GlobalState &global_state) {
    State state = convert_global_state(global_state);
    return compute_heuristic(state);
}
    
    
int AdditiveCartesianHeuristic::compute_current_order_heuristic(const GlobalState &global_state){
    //cout << "Current Order values: ";
    State state = convert_global_state(global_state);
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
    //cout << "-----------------------------" << endl;
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
            int sum = sums_h[i] + values[i]; 
            if (sum == INF)
                return DEAD_END;
            
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
    
    //cout << endl;
    //cout << "---> " << max << " order: " << pos_max << endl;
    //cout << "-----------------------------" << endl;
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
    vector<int> values;
    int sum_h = 0;
    for (const CartesianHeuristicFunction *function : heuristic_functions) {
        int value = function->get_value(state, order);
        assert(value >= 0);
        if (value == INF)
            return values;
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
    
bool AdditiveCartesianHeuristic::online_Refine(const GlobalState &global_state, std::vector<std::pair<GlobalState, int>> succStates, int* scp_order){
   // cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
   //TODO delete order if not usefull
   //TODO delete abstraction if not usefull
        
    bool debug = false;
    if(debug){
        cout << "----------- Check Refine --------- " << endl;   
        cout << "Refine State: " << global_state.get_id() << endl;
        int i = 0;
        for(int n : global_state.get_values()){
            cout << i << "=" << n << " " ;
            i++;
        }
        cout << endl;
    }
    
    prove_timer.resume();
    int infinity = EvaluationResult::INFTY;
    //heuristc value of currently expanded state
    vector<int> h_values = compute_individual_heuristics_of_order(global_state, current_order);
    int h_value = 0;
    if(debug)
        cout << "h value:       ";
    for(int v : h_values){
        if(debug)
            cout << v << " ";
        h_value += v;   
    }
    if(debug)
        cout << " = " << h_value << endl;
    
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
        int succ_h_value = 0;
        for(int v : succ_values){
            succ_h_value += v;   
        }
        for(uint i = 0; i < provable_h_values.size(); i++){
            succ_h_values += to_string(succ_values[i]) + " ";
            provable_h_values[i] = min(
                provable_h_values[i],
                (succ_values[i] == infinity) ? infinity : succ_values[i] + succ.second);
        }
        provable_h_value = min(
                provable_h_value,
                (succ_h_value == infinity) ? infinity : succ_h_value + succ.second);
        if(debug)
            cout << succ_h_values <<  " = " << succ_h_value << endl;
    }

    //Check if sum could be refined
    bool refine_sum = provable_h_value > h_value ? true : false;
    
    if(!refine_sum){
     if(false)
        cout << "---> not improvable" << endl;
        
     prove_timer.stop();
     return false;   
    }
    
    if(false)
        cout << "---> h(s) = " << h_value << " improvable to " << provable_h_value << endl;
    
    //Check which heuristic could be refined
    bool conflict = true;
    vector<bool> toRefine;
    //bool refine_all = true;
    string provable_h_values_s("provable h values: ");
    for(uint i = 0; i < provable_h_values.size(); i++){
        provable_h_values_s += to_string(provable_h_values[i]) + " ";
        if(provable_h_values[i] > h_values[i]){
            conflict = false;
            //if(values[i] > 0)
            toRefine.push_back(true);
            provable_h_values_s += "r ";   
        }
        else{
            //refine_all = false;
            toRefine.push_back(false);
            provable_h_values_s += "f ";
        }
    }
    if(false){
        cout << provable_h_values_s << endl;
        //cout << "Refinment pathology: " << conflict << endl;
    }
    prove_timer.stop();
    
    
    //Refinement pathology
    
    if(heuristic_functions.size() > 1 && conflict){
        /*
        merge_timer.resume();
        heuristic_functions[0]->merge(heuristic_functions[1]);
        heuristic_functions.erase(heuristic_functions.begin() + 1);
        cost_saturation->remove_abstraction(1);
        merge_timer.stop();
        cost_timer.resume();
        cost_saturation->recompute_cost_partitioning();
        cost_timer.stop();
        
        int new_h_value = compute_heuristic(global_state);
        if(h_value <  new_h_value){
            if(true)
                cout << "Merge Heuristic has been improved h_old(s) = " << h_value << " --> h_new(s) = " << new_h_value << endl;
           return true;
        }
        if(debug)
            cout << "Heuristic not improved" << endl;
        */
    }
    
    
    
   //CHANGE SCP ORDER
    cost_timer.resume();
    //cost_saturation->recompute_cost_partitioning(current_order);  //TODO WHY
    vector<int> updated_order(cost_saturation->get_order(current_order));
    //cout << "Current order: " << current_order << endl;
    //compute_current_order_heuristic(global_state);
    if(false){
        cout << "----------------------------" << endl;
        cout << "Old order: " << current_order << endl;
        for(int o : updated_order){
            cout << o << " ";   
        }
        cout << endl;
    }
    
    int new_pos = (*rng_order)(updated_order.size() - 1);
    new_pos = new_pos == 0 ? 1 : new_pos;
    iter_swap(updated_order.begin(), updated_order.begin() + (new_pos)); 
    
    //REORDER move all abstractions where state satiesfies the goal to the end
    //REVERS all other abstractions
    State state = convert_global_state(global_state);
    vector<int> new_order;
    vector<int> sat_abs;
    //std::vector<int>::reverse_iterator rit = updated_order.rbegin();
    /*for (; rit!= updated_order.rend(); ++rit){
           CartesianHeuristicFunction *function = *(heuristic_functions.begin() + *rit);
           if(function->satisfies_goal(state)){
                sat_abs.push_back(*rit);
           }
           else{
               new_order.push_back(*rit);
           }
    }*/
    for(int pos : updated_order){
         CartesianHeuristicFunction *function = heuristic_functions[pos];
        if(function->satisfies_goal(state)){
                sat_abs.push_back(pos);
           }
           else{
               new_order.push_back(pos);
           }
    }
    //Shuffel new_order
    //shuffle(new_order.begin(), new_order.end(), default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
    new_order.insert( new_order.end(), sat_abs.begin(), sat_abs.end() );
    //cout << "Recompute for new selected order: " << endl;
    cost_saturation->recompute_cost_partitioning(new_order);
    if(false){
        //cout << "----------------------------" << endl;
        cout << "new order: "  << endl;
        for(int o : new_order){
            cout << o << " ";   
        }
        cout << endl;
    }
    
    
   //Check if heuristic has been improved
   //cost_saturation->recompute_cost_partitioning(updated_order);
   int new_h_value = compute_current_order_heuristic(global_state);
   if(h_value <  new_h_value){
       if(true)
            cout << "Order " << current_order << " Heuristic has been improved h_old(s) = " << h_value << " --> h_new(s) = " << new_h_value << endl;
       improved_order++;
       
       //Update h_values 
       int new_scp_order = current_order;
       bool exists = cost_saturation->add_order(new_order, &new_scp_order);
        //cout << "new order id: " << new_scp_order << " old order id: " << current_order << endl;
       if(!exists){
            cout << "New order added" << endl;
            *scp_order = new_scp_order;
            number_of_orders++;
            usefullnes_of_order.push_back(0);
            lifetime_of_order.push_back(0);
            for (CartesianHeuristicFunction *function : heuristic_functions) {
                function->update_h_and_g_values(new_scp_order, true);
            }
       }
       cost_timer.stop();
       return true;
   }   
    
    cost_timer.stop();
   //should not derease h values do not change
   /*
   if(h_value >  new_h_value){
      decreased_order++;
      cout << "!!!! Order Heuristic decreased h_old(s) = " << h_value << " --> h_new(s) = " << new_h_value << endl;
   }
   //If the order change has not increased the heuristic the old order is used !! h_values vector has not changed
   if(h_value == new_h_value){
       cout << "Order Heuristic stayed h_old(s) = " << h_value << " --> h_new(s) = " << new_h_value << endl;
       //cost_saturation->recompute_cost_partitioning(*scp_order);
       //cout << "reset to order: " << *scp_order << endl;;
       
   }
   */
    
   //Update current h value
   //h_value = new_h_value;
   if(false){
        cout << "Heuristic not improved" << endl;    
        cout << "++++++++++ Refine all Heuristics +++++++++++" << endl;
   }  
       
    //refine every heuristic  
    refine_timer.resume();
    bool refined = false;
    if(max_states_online - online_refined_states > 0){
        
        State state = convert_global_state(global_state);
        
        for(size_t i = 0; i < toRefine.size(); i++){
            if(false)
                cout << "------------Refine " << i << " ----------------------" << endl;
            if(toRefine[i]){              
               int refined_states = heuristic_functions[i]->online_Refine(state, max_iter, update_h_values, max_states_online - online_refined_states);
               if(refined_states > 0){
                    refined = true;  
                    if(false)
                        cout << "--> Refined" << endl;
               }
               online_refined_states += refined_states;           
            }
        }
        

        if(refined){
            if(false)
                cout << "recompute cost partitioning" << endl;
            //cost_timer.resume();
            //cost_saturation->recompute_cost_partitioning(*scp_order);
            //cost_saturation->recompute_cost_partitioning(); //!!!! ONLY USE FREE REMAINING COST
            //Update for all orders all h values
            for(int o = 0; o < number_of_orders; o++){
                //cout << "Order: "  << o << endl;
                cost_saturation->recompute_cost_partitioning_unused(o);
                for (CartesianHeuristicFunction *function : heuristic_functions) {
                    function->update_h_and_g_values(o, false);
                }   
            }
            //Reset Refined status
            for (CartesianHeuristicFunction *function : heuristic_functions) {
                function->set_refined(false);   
            }
            
            //cost_timer.stop();
        }
 
        //cout << "Recompute: " << endl;
        new_h_value = compute_heuristic(global_state);
       if(h_value <  new_h_value){
           improved_refine++;
           if(true)    
                cout << "Refine Heuristic wither order " << current_order << " has been improved h_old(s) = " << h_value << " --> h_new(s) = " << new_h_value << endl;
           refine_timer.stop();
           return true;
       }
        /*
       if(h_value >  new_h_value){       
          cout << "!!!! Refine Heuristic decreased h_old(s) = " << h_value << " --> h_new(s) = " << new_h_value << endl;
       }
       */
        if(false)
            cout << "REFINE heuristic not improved" << endl;
    }
    refine_timer.stop();
       
   //TODO decide which abstraction should be merged
   /*
   if(heuristic_functions.size() > 1 && ){
    merge_timer.resume();
    heuristic_functions[0]->merge(heuristic_functions[1]);
    heuristic_functions.erase(heuristic_functions.begin() + 1);
    cost_saturation->remove_abstraction(1);
    merge_timer.stop();
    cost_timer.resume();
    cost_saturation->recompute_cost_partitioning();
    cost_timer.stop();
   }
   */ 
      
   return refined;  
}
    
void AdditiveCartesianHeuristic::change_to_order(int id){
    //cout << "Change to order: " << id << endl;
    if(current_order == id){
        return;   
    }
    current_order = id;
    cost_saturation->recompute_cost_partitioning(id);
}
    
	
void AdditiveCartesianHeuristic::print_statistics(){
        cout << "Order: ";
        print_order();
		for (const CartesianHeuristicFunction *function : heuristic_functions) {
			function->print_statistics();	
		}
        cout << endl;
        cout << "Refined States: " << online_refined_states << endl;
        cout << "Improved States order: " << improved_order << endl;
        cout << "Decreased States order: " << decreased_order << endl;
        cout << "Improved States refine: " << improved_refine << endl;
        cout << "cost time: " << cost_timer << endl;
        cout << "refine time: " << refine_timer << endl;
        cout << "merge time: " << merge_timer << endl;
        cout << "prove time: " << prove_timer << endl;
        cout << "Heuristic evaluation time: " << update_timer << endl;
        cout << "Get values timer: " << values_timer << endl;
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
        for(uint i = 0; i < usefullnes_of_order.size(); i++){
            cout << (float)usefullnes_of_order[i] / lifetime_of_order[i] << " ";    
        }
        cout << endl;
    
        cout << "usefullnes of abstraction: " << endl;
        for(int u : usefullnes_of_abstraction){
            cout << u << " ";   
        }
    
        cout << endl;
        cost_saturation->print_statistics();
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
