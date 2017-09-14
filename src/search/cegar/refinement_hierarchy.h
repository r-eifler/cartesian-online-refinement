#ifndef CEGAR_REFINEMENT_HIERARCHY_H
#define CEGAR_REFINEMENT_HIERARCHY_H

#include "abstract_state.h"  //new

#include <cassert>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>


class State;

namespace cegar {
class Node;

/*
  This class stores the refinement hierarchy of a Cartesian
  abstraction. The hierarchy forms a DAG with inner nodes for each
  split and leaf nodes for the abstract states.

  It is used for efficient lookup of heuristic values during search.

  Inner nodes correspond to abstract states that have been split (or
  helper nodes, see below). Leaf nodes correspond to the current
  (unsplit) states in an abstraction. The use of helper nodes makes
  this structure a directed acyclic graph (instead of a tree).
*/
class RefinementHierarchy {
    std::unique_ptr<Node> root;

public:
    RefinementHierarchy();

    // Visual Studio 2013 needs an explicit implementation.
    RefinementHierarchy(RefinementHierarchy &&other)
        : root(std::move(other.root)) {
    }

    Node *get_node(const State &state) const;

    Node *get_root() const {
        return root.get();
    }
	
	std::vector<std::pair<int, std::vector<int>>> get_split_vars(AbstractState* state);
};


class Node {
    static const int LEAF_NODE = -1;
    /*
      While right_child is always the node of a (possibly split)
      abstract state, left_child may be a helper node. We add helper
      nodes to the hierarchy to allow for efficient lookup in case more
      than one fact is split off a state.
    */
    // TODO: Use shared_ptr for left_child and unique_ptr for right_child?
    Node *left_child;
    Node *right_child;

    // Variable and value for which the corresponding state was split.
    int var;
    int value;

    // Estimated cost to nearest goal state from this node's state.
    int h;
	
  
    //Corresponding abstract state TODO
    AbstractState *abstract_state;

public:
    Node();
    ~Node();

    Node(const Node &) = delete;
    Node &operator=(const Node &) = delete;

    /*
      Update the split tree for the new split. Additionally to the left
      and right child nodes add |values|-1 helper nodes that all have
      the right child as their right child and the next helper node as
      their left child.
    */
    std::pair<Node *, Node *> split(int var, const std::vector<int> &values);

    bool is_split() const {
        assert((!left_child && !right_child &&
                var == LEAF_NODE && value == LEAF_NODE) ||
               (left_child && right_child &&
                var != LEAF_NODE && value != LEAF_NODE));
        return left_child;
    }

    bool owns_right_child() const {
        assert(is_split());
        return !left_child->is_split() || left_child->right_child != right_child;
    }

    int get_var() const {
        assert(is_split());
        return var;
    }

    Node *get_child(int value) const;

    void increase_h_value_to(int new_h) {
		//if(new_h < h)
		//	std::cout << "NODE: old h: " << h << " new h: " << new_h << std::endl;
        //assert(new_h >= h);  TODO
        h = new_h;
    }

    int get_h_value() const {
        return h;
    }
	
	void update_h_value(int pos, int value){
		//assert((uint) pos < h_values.size());
		//h_values[pos] = value;
		abstract_state->set_h_value(pos, value);
	}
	
	int add_h_value(int value){
		//h_values.push_back(value);
		return abstract_state->add_h_value(value);
	}
	
	
	int get_h_value(int pos) const {
		//assert((uint) pos < h_values.size());
        //return h_values[pos];
		return abstract_state->get_h_value(pos);
    }
	
	std::vector<int> get_h_values() const {
		//return h_values;	
		return abstract_state->get_h_values();
	}
  
	void set_c_h(int v){
		abstract_state->set_c_h(v);	
	}
	
	int get_c_h(){
		return abstract_state->get_c_h();	
	}
  
    //TODO
    void set_AbstractState(AbstractState *state){
      abstract_state = state; 
    }
  
    AbstractState *get_AbstractState(){
      return  abstract_state;
    }
	
	void get_split_vars(AbstractState* state, std::vector<std::pair<int, std::vector<int>>>* splits){
		//std::cout << "..................................." << std::endl;
		if(! is_split()){
			return;	
		}
		/*
		if(state->contains(var, value) && state->count(var) > 1){
			splits->push_back(std::make_pair(var, value));
		}*/
		std::vector<int> values;
		//traverse all left nodes as long they have the same right child as the current node
		Node* lc = left_child;

		/*
		while(lc->right_child == right_child){
				assert(var == lc->var);
				std::cout << lc->var << " ";
				values.push_back(lc->value);
				lc = lc->left_child;
		}
		*/
		//std::cout << std::endl;
		
		if( !(state->contains(var, value) && state->count(var) > 1)){
			//std::cout << "right" << std::endl;
			right_child->get_split_vars(state, splits);			
		}
		else{
			//std::cout << "left" << std::endl;
			//std::cout << "var " << var << " = " << value << " " << std::endl;
			values.push_back(value);
			splits->push_back(make_pair(var, values));
			lc->get_split_vars(state, splits);
		}
	}
	
	std::pair<int, std::vector<int>> get_wanted_vars(AbstractState* state, Node** lc, Node** rc){
		std::pair<int, std::vector<int>> wanted;
		//leaf reached 
		if(! is_split()){
			return wanted;	
		}
		//std::cout << "State: " << *state << std::endl;
		//std::cout << "var " << var << " = " << value << std::endl;
		//if the state still cantaines more than the value which needs to split
		if(state->contains(var, value) && state->count(var) > 1){
			//collect all values of the left childs which have the same right child as the current node
			std::vector<int> values;
			values.push_back(value);
			Node * cn = left_child;
			while(cn->right_child == right_child){
				//if a value is not contained in the state it can not bee split anymore 
				if(state->contains(var, cn->value)){
					values.push_back(cn->value);
				}
				cn = cn->left_child;
			}
			//if the state only containes the wanted vars a split is not neccesary
			if((int)values.size() >= state->count(var)){
				return right_child->get_wanted_vars(state, lc, rc);
			}
			/*
			std::cout << "Wanted: ";
			for(int v : values){
				std::cout << v << " ";	
			}
			std::cout << std::endl;
			*/
			wanted = make_pair(var, values);		
			*lc = cn;
			*rc = right_child;	
			
			return wanted;
		}
		if(state->contains(var, value) && state->count(var) == 1){
			return right_child->get_wanted_vars(state, lc, rc);			
		}
		//if the value is not contained in the state the next split is defined by the left child
		if(!state->contains(var, value)){
			return left_child->get_wanted_vars(state, lc, rc);			
		}
		
		return wanted;
	}
	
};
}

#endif
