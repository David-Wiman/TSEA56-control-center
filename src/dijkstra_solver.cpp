#include "dijkstra_solver.h"
#include "map_node.h"

#include <list>
#include <string>

using namespace std;

DijkstraSolver::DijkstraSolver() {

}

void DijkstraSolver::update_map(MapGraph *m) {
    map_graph = m
}

friend std::list<MapNode*>DijkstraSolver::solve(string name) {
    
    
}