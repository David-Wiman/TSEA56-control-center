#ifndef DIJKSTRA_SOLVER_H
#define DIJKSTRA_SOLVER_H

#include "map_node.h"

#include <list>
#include <string>

class DijkstraSolver {
public:
    DijkstraSolver();
    ~DijkstraSolver();

    void update_map(MapGraph *map_graph);
    std::list<MapNode*> solve(std::string name);

private:
    MapGraph *map_graph;

};

#endif // DIJKSTRA_SOLVER_H