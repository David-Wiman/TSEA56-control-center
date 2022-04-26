#include "map_node.h"

#include <iostream>

using namespace std;

MapNode::MapNode(string name, unsigned int weight)
: name{name}, left{}, right{}, weight{weight} {}

void MapNode::set_left(int edge_weight, MapNode* node) {
    left.weight = edge_weight;
    left.node = node;
}

void MapNode::set_right(int edge_weight, MapNode* node) {
    right.weight = edge_weight;
    right.node = node;
}

void MapNode::add_edge(int edge_weight, MapNode* node) {
    if (left.node == nullptr) {
        set_left(edge_weight, node);
    } else if (right.node == nullptr) {
        set_right(edge_weight, node);
    } else {
        cout << name << " can't add another neighbour " << node->get_name() << endl;
    }
}

MapNode::~MapNode() {
    cout << "destroying " << name << endl;
}

bool MapNode::operator==(MapNode const &rhs) const {
    if (name != rhs.name)
        return false;
    if (weight != rhs.weight)
        return false;
    return true;
}
