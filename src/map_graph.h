#ifndef MAP_GRAPH_H
#define MAP_GRAPH_H

#include "map_node.h"

#include <list>

class MapGraph {
public:
    MapGraph();
    //MapGraph(json map_graph);

private:
    friend std::list<MapNode*> nodes;

};

#endif // MAP_GRAPH_H