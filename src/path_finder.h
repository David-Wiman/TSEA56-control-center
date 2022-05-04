/*
 * Use PathFinder() + update_map(json m) +
 * solve(std::string start_node_name) + get_drive_mission()
 * OR
 * Use PathFinder(json m, std::string start_node_name) +
 * get_drive_mission()
 * OR
 * Use PathFinder(list<MapNode*> map_nodes, std::string start_node_name) +
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

class PathFinder {
public:
    PathFinder();
    PathFinder(json m, std::string start_node_name);
    PathFinder(std::list<MapNode*> m, std::string start_node_name);
    ~PathFinder();

    PathFinder(PathFinder const&) = delete;
    PathFinder operator=(PathFinder const&) = delete;

    void solve(std::string start_node_name);
    void solve(std::string start_node_name, std::string stop_node_name);
    void find_path(MapNode *neighbour, std::string stop_node_name);
    std::vector<int> get_drive_mission();
    void update_map(json m);
    void make_MapNode_list(json m);

    void reset_progress();
    void done_with_drive_instruction();
    int get_nodes_passed();
    int get_current_drive_instruction();
    std::string get_current_road_segment();

private:
    MapNode *initiate_map_graph(std::string &start_node_name);
    std::list<MapNode*> nodes{};
    std::vector<int> drive_mission{};
    int nodes_passed{};

};

#endif // DIJKSTRA_SOLVER_H