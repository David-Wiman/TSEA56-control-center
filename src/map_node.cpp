#include "map_node.h"

#include <iostream>

using namespace std;

MapNode::MapNode(string name, MapNode *next_left, MapNode *next_right, unsigned int weight)
: name{name}, next_left{next_left}, next_right{next_right}, weight{weight} {}

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
