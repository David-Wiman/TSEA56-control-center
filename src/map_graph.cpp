#include "map_graph.h"

#include <list>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

MapGraph::MapGraph(): nodes{} {
    MapNode *map_nodeA = new MapNode{"A"};
    MapNode *map_nodeB = new MapNode{"B"};
    MapNode *map_nodeC = new MapNode{"C"};
    MapNode *map_nodeD = new MapNode{"D"};

    map_nodeA->set_left(3, map_nodeB);
    map_nodeA->set_right(1, map_nodeC);
    map_nodeB->set_left(2, map_nodeD);
    map_nodeC->set_left(1, map_nodeB);
    map_nodeC->set_right(5, map_nodeD);

    nodes.push_back(map_nodeA);
    nodes.push_back(map_nodeB);
    nodes.push_back(map_nodeC);
    nodes.push_back(map_nodeD);
}

/*MapGraph::MapGraph(json map_graph): nodes{} {
    for (auto const& node : map_graph) {
        MapNode *map_node = new MapNode{node.key()};
        nodes.push_back(map_node);
    }
}*/