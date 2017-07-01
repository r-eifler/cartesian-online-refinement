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
	
	std::vector<std::pair<int, int>> get_split_vars(AbstractState* state);
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
  
  
    //TODO
    void set_AbstractState(AbstractState *state){
      abstract_state = state; 
    }
  
    AbstractState *get_AbstractState(){
      return  abstract_state;
    }
	
	void get_split_vars(AbstractState* state, std::vector<std::pair<int, int>>* splits){
		if(! is_split()){
			return;	
		}
		if(state->contains(var, value) && !(state->count(var) == 1)){
			splits->push_back(std::make_pair(var, value));
		}
		
		left_child->get_split_vars(state, splits);
		right_child->get_split_vars(state, splits);
	}
	
};
}

#endif
