#include "dijkstra_solver.h"
#include "map_node.h"

#include <list>
#include <string>

using namespace std;

DijkstraSolver::DijkstraSolver(): map_graph{} {}

void DijkstraSolver::update_map(MapGraph m) {
    map_graph = m;
}

std::list<MapNode*> DijkstraSolver::solve(string start_node) {
    list<MapNode*> optimal_route;
    initiate_map_graph(start_node);

    return optimal_route;
}

void DijkstraSolver::initiate_map_graph(string start_node) {
    for (MapNode* node : map_graph.nodes) {
        if (node->get_name() == start_node) {
            node->set_weight(0);
        } else {
            node->set_weight(INT_MAX);
        }
    }
}