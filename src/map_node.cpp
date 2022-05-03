#include "map_node.h"
#include "log.h"

#include <iostream>

using namespace std;

MapNode::MapNode(string name, unsigned int weight)
: name{name}, left{}, right{}, weight{weight} {}

void MapNode::set_left(int edge_weight, MapNode *node) {
    left.weight = edge_weight;
    left.node = node;
}

void MapNode::set_right(int edge_weight, MapNode *node) {
    right.weight = edge_weight;
    right.node = node;
}

void MapNode::add_edge(int edge_weight, MapNode *node) {
    if (left.node == nullptr) {
        cout << get_name() << " left: " << node->get_name() << endl;
        set_left(edge_weight, node);
    } else if (right.node == nullptr) {
        cout << get_name() << " right: " << node->get_name() << endl;
        set_right(edge_weight, node);
    } else {
        Logger::log(WARNING, "map_node.cpp", "add_edge", "Try to add edge to non-existent node");
    }
}

MapNode::~MapNode() {}

bool MapNode::operator==(MapNode const &rhs) const {
    if (name != rhs.name)
        return false;
    if (weight != rhs.weight)
        return false;
    return true;
}
