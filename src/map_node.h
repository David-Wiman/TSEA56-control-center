#ifndef MAP_NODE_H
#define MAP_NODE_H

#include <iostream>
#include <climits>

/* To stop errors...*/
class MapNode;

struct Edge {
    int weight = INT_MAX;
    MapNode *node = nullptr;
};

class MapNode {
public:
    MapNode(std::string name, unsigned int weight = UINT_MAX);
    void set_left(int edge_weight, MapNode *node);
    void set_right(int edge_weight, MapNode *node);
    void add_edge(int edge_weight, MapNode *node);
    ~MapNode();

    MapNode(MapNode const&);
    MapNode operator=(MapNode const&) = delete;

    bool operator==(MapNode const &other) const;

    void set_weight(int w) {
        weight = w;
    }
    void set_visited(bool value) {
        visited = value;
    }
    unsigned int get_weight() const {
        return weight;
    }
    Edge get_left() const {
        return left;
    }
    Edge get_right() const {
        return right;
    }
    std::string get_name() const {
        return name;
    }
    bool is_visited() const {
        return visited;
    }
    void set_parent_node(MapNode *parent) {
        parent_node = parent;
    }
    MapNode* get_parent_node() {
        return parent_node;
    }
    void set_child_node(MapNode *child) {
        child_node = child;
    }
    MapNode* get_child_node() {
        return child_node;
    }

private:
    MapNode *parent_node{};
    MapNode *child_node{nullptr};
    bool visited{};
    std::string name;
    Edge left;
    Edge right;
    unsigned int weight;
};

#endif // MAP_NODE_H