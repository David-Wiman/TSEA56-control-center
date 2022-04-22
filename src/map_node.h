#ifndef MAP_NODE_H
#define MAP_NODE_H

#include <iostream>
#include <climits>

/* To stop errors...*/
class MapNode;

struct Edge {
    int weight = INT_MAX;
    MapNode* node = nullptr;
};

class MapNode {
public:
    MapNode(std::string name, unsigned int weight = __INT_MAX__);
    void set_left(int edge_weight, MapNode* node);
    void set_right(int edge_weight, MapNode* node);
    ~MapNode();

    MapNode(MapNode const&) = delete;
    MapNode operator=(MapNode const&) = delete;

    bool operator==(MapNode const &other) const;

    void set_weight(int w) {
        weight = w;
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

private:
    std::string name;
    Edge left;
    Edge right;
    unsigned int weight;
};

#endif // MAP_NODE_H