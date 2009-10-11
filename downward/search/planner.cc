#include "a_star_search.h"
#include "best_first_search.h"
#include "cyclic_cg_heuristic.h"
#include "cg_heuristic.h"
#include "ff_heuristic.h"
#include "fd_heuristic.h"
#include "lm_cut_heuristic.h"
#include "max_heuristic.h"
#include "additive_heuristic.h"
#include "goal_count_heuristic.h"
#include "blind_search_heuristic.h"
#include "globals.h"
#include "operator.h"
#include "timer.h"
#include "general_eager_best_first_search.h"
#include "landmarks/landmarks_graph.h"
#include "landmarks/landmarks_graph_rpg_sasp.h"
#include "landmarks/landmarks_count_heuristic.h"
#include "landmarks/exploration.h"
#include "hm_heuristic.h"

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

int save_plan(const vector<const Operator *> &plan);

int main(int argc, const char **argv) {
    bool poly_time_method = false;

    bool a_star_search = false;
    bool cg_heuristic = false, cg_preferred_operators = false;
    bool cyclic_cg_heuristic = false, cyclic_cg_preferred_operators = false;
    bool ff_heuristic = false, ff_preferred_operators = false;
    bool additive_heuristic = false, additive_preferred_operators = false;
    bool fd_heuristic = false;
    bool hsp_max_heuristic = false;
    bool goal_count_heuristic = false;
    bool blind_search_heuristic = false;
    bool lm_cut_heuristic = false;
    bool use_gen_search = false;
    bool lm_heuristic = false;
    bool lm_heuristic_admissible = false;
    bool lm_preferred = false;
    bool use_hm = false;



    for(int i = 1; i < argc; i++) {
	for(const char *c = argv[i]; *c != 0; c++) {
            if(*c == 'o') {
                a_star_search = true; // "o"ptimal
            } else if(*c == 'c') {
		cg_heuristic = true;
	    } else if(*c == 'C') {
		cg_preferred_operators = true;
            } else if(*c == 'y') {
                cyclic_cg_heuristic = true;
            } else if(*c == 'Y') {
                cyclic_cg_preferred_operators = true;
	    } else if(*c == 'f') {
		ff_heuristic = true;
	    } else if(*c == 'F') {
		ff_preferred_operators = true;
	    } else if(*c == 'a') {
		fd_heuristic = true;
	    } else if(*c == 'k') {
	    use_gen_search = true;
	    } else if(*c == 'm') {
		hsp_max_heuristic = true;
	    } else if(*c == 'h') {
	    use_hm = true;
	    } else if(*c == 'd') {
		additive_heuristic = true;
	    } else if(*c == 'l') {
	    lm_heuristic = true;
	    } else if(*c == 's') {
	    lm_heuristic_admissible = true;
	    } else if(*c == 'L') {
	    lm_preferred = true;
	    } else if(*c == 'D') {
		additive_preferred_operators = true;
	    } else if(*c == 'g') {
		goal_count_heuristic = true;
	    } else if(*c == 'b') {
		blind_search_heuristic = true;
            } else if(*c == 'u') {
                lm_cut_heuristic = true;
            } else if(*c >= '0' && *c <= '9') {
                g_abstraction_max_size = ::atoi(c);
                while(*c >= '0' && *c <= '9')
                    c++;
                c--;
                if(g_abstraction_max_size < 1) {
                    cerr << "error: abstraction size must be at least 1"
                         << endl;
                    return 2;
                }
	    } else if(*c == 'A') {
	        c++;
                g_abstraction_nr = ::atoi(c);
                while(*c >= '0' && *c <= '9')
                    c++;
                c--;
	    } else if(*c == 'R') {
	        c++;
                int seed = ::atoi(c);
                while(*c >= '0' && *c <= '9')
                    c++;
                c--;
                cout << "random seed: " << seed << endl;
                srand(seed);
            } else if(*c == 'S') {
                const char *arg = c;
                c++;
                g_compose_strategy = *c++ - '1';
                if(g_compose_strategy < 0 ||
                   g_compose_strategy >= MAX_COMPOSE_STRATEGY) {
                    cerr << "Unknown option: " << arg << endl;
                    return 2;
                }
                g_collapse_strategy = *c++ - '1';
                if(g_collapse_strategy < 0 ||
                   g_collapse_strategy >= MAX_COLLAPSE_STRATEGY) {
                    cerr << "Unknown option: " << arg << endl;
                    return 2;
                }
                if(*c == '1' || *c == '2') {
                    if(*c == '2')
                        g_merge_and_shrink_bound_is_for_product = false;
                    c++;
                }
                c--;
	    } else {
		cerr << "Unknown option: " << *c << endl;
		return 2;
	    }
	}
    }

    if(fd_heuristic) {
        cout << "Composition strategy: ";
        if(g_compose_strategy == COMPOSE_LINEAR_CG_GOAL_LEVEL) {
            cout << "linear CG/GOAL, tie breaking on level (main)";
        } else if(g_compose_strategy == COMPOSE_LINEAR_CG_GOAL_RANDOM) {
            cout << "linear CG/GOAL, tie breaking random";
        } else if(g_compose_strategy == COMPOSE_LINEAR_GOAL_CG_LEVEL) {
            cout << "linear GOAL/CG, tie breaking on level";
        } else if(g_compose_strategy == COMPOSE_LINEAR_RANDOM) {
            cout << "linear random";
        } else if(g_compose_strategy == COMPOSE_DFP) {
            cout << "Draeger/Finkbeiner/Podelski";
        }
        cout << endl;
        if(g_compose_strategy == COMPOSE_DFP) {
            cerr << "DFP composition strategy not implemented." << endl;
            return 2;
        }

        cout << "Collapsing strategy: ";
        if(g_collapse_strategy == COLLAPSE_HIGH_F_LOW_H) {
            cout << "high f/low h (main)";
        } else if(g_collapse_strategy == COLLAPSE_LOW_F_LOW_H) {
            cout << "low f/low h";
        } else if(g_collapse_strategy == COLLAPSE_HIGH_F_HIGH_H) {
            cout << "high f/high h";
        } else if(g_collapse_strategy == COLLAPSE_RANDOM) {
            cout << "random states";
        } else if(g_collapse_strategy == COLLAPSE_DFP) {
            cout << "Draeger/Finkbeiner/Podelski";
        }
        cout << endl;
    }

    if(!cg_heuristic && !cyclic_cg_heuristic
       && !ff_heuristic && !additive_heuristic && !goal_count_heuristic
       && !blind_search_heuristic && !fd_heuristic && !hsp_max_heuristic
       && !lm_cut_heuristic && !lm_heuristic && !use_hm) {
	cerr << "Error: you must select at least one heuristic!" << endl
	     << "If you are unsure, choose options \"cCfF\"." << endl;
	return 2;
    }

    istream &in = cin;
    //ifstream in("../../results/preprocess/blocks/probBLOCKS-7-0.pddl/output");

    in >> poly_time_method;
    if(poly_time_method) {
	cout << "Poly-time method not implemented in this branch." << endl;
	cout << "Starting normal solver." << endl;
    }

    read_everything(in);
    // dump_everything();


    //cout << "Generating landmarks" << endl;
    //LandmarksGraph *lm_graph = new LandmarksGraphNew(true, true, false, false);
    //LandmarksGraph *lm_graph = new LandmarksGraphNew(new Exploration);
    //lm_graph->read_external_inconsistencies();
    //lm_graph->generate();
    //cout << "Generated " << lm_graph->number_of_landmarks() << " landmarks and " << lm_graph->number_of_edges() << " orderings" << endl;

    SearchEngine *engine = 0;
    if(a_star_search) {
    	if (use_gen_search)
    		engine = new GeneralEagerBestFirstSearch(1, 1, GeneralEagerBestFirstSearch::h, true);
    	else
    		engine = new AStarSearchEngine;
    }
    else {
    	if (use_gen_search)
    	    engine = new GeneralEagerBestFirstSearch(0, 1, GeneralEagerBestFirstSearch::fifo, false);
    	else
    		engine = new BestFirstSearchEngine;
    }
    if(cg_heuristic || cg_preferred_operators)
	engine->add_heuristic(new CGHeuristic, cg_heuristic,
			      cg_preferred_operators);
    if(cyclic_cg_heuristic || cyclic_cg_preferred_operators)
	engine->add_heuristic(new CyclicCGHeuristic, cyclic_cg_heuristic,
			      cyclic_cg_preferred_operators);
    if(additive_heuristic || additive_preferred_operators)
	engine->add_heuristic(new AdditiveHeuristic, additive_heuristic,
                              additive_preferred_operators);
    if(ff_heuristic || ff_preferred_operators)
	engine->add_heuristic(new FFHeuristic, ff_heuristic,
			      ff_preferred_operators);
    if(goal_count_heuristic)
	engine->add_heuristic(new GoalCountHeuristic, true, false);
    if(blind_search_heuristic)
	engine->add_heuristic(new BlindSearchHeuristic, true, false);
    if(fd_heuristic)
	engine->add_heuristic(new FinkbeinerDraegerHeuristic, true, false);
    if(hsp_max_heuristic)
	engine->add_heuristic(new HSPMaxHeuristic, true, false);
    if(lm_cut_heuristic)
    engine->add_heuristic(new LandmarkCutHeuristic, true, false);
    if(use_hm)
    engine->add_heuristic(new HMHeuristic(2), true, false);
    if(lm_heuristic) {
    engine->add_heuristic(new LandmarksCountHeuristic(lm_preferred, lm_heuristic_admissible), true, lm_preferred);
    }

    Timer search_timer;
    engine->search();
    search_timer.stop();
    g_timer.stop();
    if(engine->found_solution())
	save_plan(engine->get_plan());

    engine->statistics();
    if(cg_heuristic || cg_preferred_operators) {
	cout << "Cache hits: " << g_cache_hits << endl;
	cout << "Cache misses: " << g_cache_misses << endl;
    }
    cout << "Search time: " << search_timer << endl;
    cout << "Total time: " << g_timer << endl;

    return engine->found_solution() ? 0 : 1;
}

int save_plan(const vector<const Operator *> &plan) {
    ofstream outfile;
    int plan_cost = 0;
    outfile.open("sas_plan", ios::out);
    for(int i = 0; i < plan.size(); i++) {
    	cout << plan[i]->get_name() << " (" << plan[i]->get_cost() << ")" << endl;
    	outfile << "(" << plan[i]->get_name() << ")" << endl;
    	plan_cost += plan[i]->get_cost();
    }
    outfile.close();
    cout << "Plan length: " << plan.size() << " step(s)." << endl;
    cout << "Plan cost: " << plan_cost << endl;
    return plan_cost;
}
