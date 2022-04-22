#ifndef DIJKSTRA_SOLVER_H
#define DIJKSTRA_SOLVER_H

#include "map_node.h"
#include "map_graph.h"

#include <list>
#include <string>

class DijkstraSolver {
public:
    DijkstraSolver();
    ~DijkstraSolver();

    DijkstraSolver(DijkstraSolver const&) = delete;
    DijkstraSolver operator=(DijkstraSolver const&) = delete;

    void update_map(MapGraph map_graph);
    std::list<MapNode*> solve(std::string start_node_name);

private:
    MapNode* initiate_map_graph(std::string start_node_name);
    MapGraph map_graph;

};

#endif // DIJKSTRA_SOLVER_H