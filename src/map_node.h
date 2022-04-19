#include <iostream>

class MapNode {
public:
    MapNode(std::string name, MapNode *next_left, MapNode *next_right, unsigned int weight = __INT_MAX__);

    MapNode(const MapNode&) = delete;
    MapNode operator=(const MapNode&) = delete;

    ~MapNode();
    void set_weight(int w) {
        weight = w;
    }
    unsigned int get_weight() {
        return weight;
    }
    MapNode* get_next_left() {
        return next_left;
    }
     MapNode* get_next_right() {
        return next_right;
    }


private:
    std::string name;
    MapNode *next_left;
    MapNode *next_right;
    unsigned int weight;
};
