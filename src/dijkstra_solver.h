/*
 * Use DijkstraSolver() + update_map(json m) + 
 * solve(std::string start_node_name) + get_drive_mission()
 * OR
 * Use DijkstraSolver(json m, std::string start_node_name) +
 * get_drive_mission()
 * OR
 * Use DijkstraSolver(list<MapNode*> map_nodes, std::string start_node_name) +
 * get_drive_mission()
 */
 
#ifndef DIJKSTRA_SOLVER_H
#define DIJKSTRA_SOLVER_H

#include "map_node.h"
#include "drive_mission_generator.h"

#include <list>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct Comparator {
    bool operator()(const MapNode *a, const MapNode *b) const {
        return a->get_weight() < b->get_weight();
    }
};

class DijkstraSolver {
public:
    DijkstraSolver();
    DijkstraSolver(json m, std::string start_node_name);
    DijkstraSolver(std::list<MapNode*> m, std::string start_node_name);
    ~DijkstraSolver();

    DijkstraSolver(DijkstraSolver const&) = delete;
    DijkstraSolver operator=(DijkstraSolver const&) = delete;

    void solve(std::string start_node_name);

    std::vector<int> get_drive_mission();
    void update_map(json m);
    void make_MapNode_list(json m);

private:
    MapNode* initiate_map_graph(std::string &start_node_name);
    std::list<MapNode*> nodes;
    std::vector<int> drive_mission{};

};

#endif // DIJKSTRA_SOLVER_H