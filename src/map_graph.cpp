#include "map_graph.h"

#include <string>
#include <list>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

MapGraph::MapGraph(): nodes{} {}

MapGraph::MapGraph(list<MapNode*> map_nodes): nodes{map_nodes} {}

/* Absolute bullshit, don't try to understand. Fills "nodes" with "MapNode" pointers */
MapGraph::MapGraph(json json_map): nodes{} {
    // Create MapNode pointers and places in "nodes"
    // Assumes json_map preserve order when iterating
    for (auto &node : json_map["Map"].items()) {
        MapNode *active_node = new MapNode{node.key()};
        nodes.push_back(active_node);
    }

    int node_index{};
    // For every node in json_map, e.g. A, B, C, ...
    for (auto &node : json_map["Map"].items()) {
        // Get node at index node_index, sets it as active_node, increases node_index
        auto active_node = nodes.begin();
        advance(active_node, node_index++);

        // For all neighbouring nodes
        for (auto &edge : node.value().items()) {
            // Get neighbours name and edge-weight
            string neighbour_name = edge.value().begin().key();
            int neightbour_distance = edge.value().begin().value();

            // Find the node that matches neighbour_name
            auto found = std::find_if(nodes.begin(), nodes.end(), [&] (MapNode *ptr) {return ptr->get_name() == neighbour_name; });
            if (found != nodes.end()) {
                // Node was found and an edge is added
                (*active_node)->add_edge(neightbour_distance, *found);
            }
        }
    }
}