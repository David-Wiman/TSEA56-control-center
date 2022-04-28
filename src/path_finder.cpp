#include "path_finder.h"
#include "map_node.h"
#include "log.h"
#include "drive_mission_generator.h"

#include <list>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

PathFinder::PathFinder() {
    Logger::log(DEBUG, "path_finder.cpp", "constructor", "PathFinder created");
}

PathFinder::PathFinder(json m, std::string start_node_name) {
    Logger::log(DEBUG, "path_finder.cpp", "constructor", "PathFinder created");
    make_MapNode_list(m);
    Logger::log(DEBUG, "path_finder.cpp", "make_MapNode_list", "Successfuly created list of MapNode*");
    solve(start_node_name);
}

PathFinder::PathFinder(list<MapNode*> map_nodes, std::string start_node_name) {
    Logger::log(DEBUG, "path_finder.cpp", "constructor", "PathFinder created");
    nodes = map_nodes;
    Logger::log(DEBUG, "path_finder.cpp", "make_MapNode_list", "Successfuly set list of MapNode*");
    solve(start_node_name);
}

PathFinder::~PathFinder() {
    for (MapNode* ptr : nodes) {
        delete ptr;
    }
}

/* Set drive_mission */
void PathFinder::solve(string start_node_name) {
    // Inititate map
    MapNode *active_node = initiate_map_graph(start_node_name);
    std::list<MapNode*> nodes_to_visit{active_node};

    // Set new starting-node and remove from list
    while (!(nodes_to_visit.empty())) {
        // Make sure the node with the lowest weight is searched first
        nodes_to_visit.sort(Comparator());
        active_node = nodes_to_visit.front();
        nodes_to_visit.pop_front();
        active_node->set_visited(true);

        // Update left neighbour's weight if bigger than active nodes weight + edge weight
        MapNode *left_neighbour = active_node->get_left().node;
        if (!(left_neighbour == nullptr)) {
            if (left_neighbour->get_weight() >= active_node->get_weight() + active_node->get_left().weight) {
                left_neighbour->set_weight(active_node->get_weight() + active_node->get_left().weight);
            }
        }
        
        // Update right neighbour's weight if bigger than active nodes weight + edge weight
        MapNode *right_neighbour = active_node->get_right().node;
        if (!(right_neighbour == nullptr)) {
            if (right_neighbour->get_weight() >= active_node->get_weight() + active_node->get_right().weight) {
                right_neighbour->set_weight(active_node->get_weight() + active_node->get_right().weight);
            }
        }

        // Add nodes to nodes_to_visit list if they are not already visited
        if (!(left_neighbour == nullptr)) {
            if (!(left_neighbour->is_visited())) {
                nodes_to_visit.push_back(left_neighbour);
            }
        } 

        if (!(right_neighbour == nullptr)) {
            if (!(right_neighbour->is_visited())) {
                nodes_to_visit.push_back(right_neighbour);
            }
        }
    }

    /* Sort nodes by increasing node-weight */
    nodes.sort(Comparator());

    DriveMissionGenerator drive_mission_generator{nodes};
    drive_mission = drive_mission_generator.get_drive_mission();
    Logger::log(DEBUG, "path_finder.cpp", "solve", "Ordered vector of drive instructions created");
}

/* Sets nodes */
void PathFinder::update_map(json m) {
    make_MapNode_list(m);
}

/* 
 * Set all non-starting-node-weights to UINT_MAX and as unvisited,
 * set starting node weight to 0. Returns starting node
 */
MapNode* PathFinder::initiate_map_graph(string &start_node_name) {
    MapNode* start_node{};
    for (MapNode* node : nodes) {
        if (node->get_name() != start_node_name) {
            node->set_weight(UINT_MAX);
            node->set_visited(false);
        } else {
            node->set_weight(0);
            start_node = node;
        }
    }
    Logger::log(DEBUG, "path_finder.cpp", "initiate_map_graph", "Map initiated");
    return start_node;
}

vector<int> PathFinder::get_drive_mission() {
    return drive_mission;
}

void PathFinder::make_MapNode_list(json json_map) {
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