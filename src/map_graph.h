#ifndef MAP_GRAPH_H
#define MAP_GRAPH_H

#include "map_node.h"

#include <list>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class MapGraph {
public:
    MapGraph();
    MapGraph(std::list<MapNode*> map_nodes);
    MapGraph(json json_map);

private:
    std::list<MapNode*> nodes;
    friend class DijkstraSolver;

};

#endif // MAP_GRAPH_H