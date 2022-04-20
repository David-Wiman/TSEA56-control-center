#ifndef MAP_NODE_H
#define MAP_NODE_H

#include <iostream>

class MapNode {
public:
    MapNode(std::string name, MapNode *next_left, MapNode *next_right, unsigned int weight = __INT_MAX__);
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
    MapNode* get_next_left() const {
        return next_left;
    }
     MapNode* get_next_right() const {
        return next_right;
    }

private:
    std::string name;
    MapNode *next_left;
    MapNode *next_right;
    unsigned int weight;
};

#endif // MAP_NODE_H