#include "dijkstra_solver.h"
#include "map_node.h"

#include <list>
#include <string>

using namespace std;

DijkstraSolver::DijkstraSolver(): map_graph{} {}

DijkstraSolver::~DijkstraSolver() {}

/* Sets a new map */
void DijkstraSolver::update_map(MapGraph m) {
    map_graph = m;
}

/* Returns a list of MapNode* in increasing node-weight order */
std::list<MapNode*> DijkstraSolver::solve(string start_node_name) {
    // Inititate map
    MapNode* active_node = initiate_map_graph(start_node_name);
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
    map_graph.nodes.sort(Comparator());

    return map_graph.nodes;
}

/* 
 * Set all non-starting-node-weights to INF_MAX and as unvisited,
 * set starting node weight to 0. Returns starting node
 */
MapNode* DijkstraSolver::initiate_map_graph(string start_node_name) {
    MapNode* start_node{};
    for (MapNode* node : map_graph.nodes) {
        if (node->get_name() != start_node_name) {
            node->set_weight(INT_MAX);
            node->set_visited(false);
        } else {
            node->set_weight(0);
            start_node = node;
        }
    }
    return start_node;
}