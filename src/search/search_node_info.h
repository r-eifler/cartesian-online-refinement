#ifndef SEARCH_NODE_INFO_H
#define SEARCH_NODE_INFO_H

#include "state_id.h"

// For documentation on classes relevant to storing and working with registered
// states see the file state_registry.h.

struct SearchNodeInfo {
    enum NodeStatus {NEW = 0, OPEN = 1, CLOSED = 2, DEAD_END = 3};

    unsigned int status : 2;
    int g : 30;
    StateID parent_state_id;
    int creating_operator;
    int real_g;
	
	int h;
	int solution = false;
	int depth = 0;

    SearchNodeInfo()
        : status(NEW), g(-1), parent_state_id(StateID::no_state),
          creating_operator(-1), real_g(-1), h(-1) {
    }
};

#endif
