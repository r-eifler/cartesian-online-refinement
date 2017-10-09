#include "refinement_hierarchy.h"

#include "../task_proxy.h"

using namespace std;

namespace cegar {
Node::Node()
    : left_child(nullptr),
      right_child(nullptr),
      var(LEAF_NODE),
      value(LEAF_NODE),
      h(0) {
}

Node::~Node() {
    if (is_split()) {
        if (owns_right_child()) {
            delete right_child;
        }
        delete left_child;
    }
}

pair<Node *, Node *> Node::split(int var, const vector<int> &values) {
    Node *helper = this;
    right_child = new Node();
    for (int value : values) {
        Node *new_helper = new Node();
        helper->var = var;
        helper->value = value;
        helper->left_child = new_helper;
        helper->right_child = right_child;
        assert(helper->is_split());
        helper = new_helper;
    }
    assert(!helper->is_split());
    return make_pair(helper, right_child);
}

Node *Node::get_child(int value) const {
    assert(is_split());
    if (value == this->value)
        return right_child;
    return left_child;
}


RefinementHierarchy::RefinementHierarchy()
    : root(new Node()) {
	eval_timer.reset();
	eval_timer.stop();
		
	for(int i = 0; i < 100; i++){
		depth_dest.push_back(0);	
	}
}
	
void RefinementHierarchy::print_average_depth(){
		cout << "RH average depth: " << (depth / eval) << std::endl;
		cout << "eval time: " << eval_timer << std::endl;
		cout << "average EVAL time: " << (eval_timer() / eval) << std::endl;
	
		cout << "depth: ";
		for(int d : depth_dest){
			cout << d << " ";
		}
		cout << std::endl;
}

Node *RefinementHierarchy::get_node(const State &state){
    assert(root);
	eval++;
	int d = 0;
	eval_timer.resume();
	eval_timer_once.reset();
	eval_timer_once.resume();
    Node *current = root.get();
    while (current->is_split()) {
        current = current->get_child(state[current->get_var()].get_value());
		depth++;
		d++;
    }
	eval_timer_once.stop();
	eval_timer.stop();
	depth_dest[d]++;
	//cout << d << " --> " << eval_timer_once() << endl;
    return current;
}
	
std::vector<std::pair<int, std::vector<int>>> RefinementHierarchy::get_split_vars(AbstractState* state){
	assert(root);
	std::vector<std::pair<int, std::vector<int>>> split_vars;
    Node *current = root.get();
    current->get_split_vars(state, &split_vars);
	//for(pair<int, int> p : split_vars){
	//	cout << "var " << p.first << " = " << p.second << endl;	
	//}
	return split_vars;
}
}
