#ifndef DRIVE_MISSION_GENERATOR_H
#define DRIVE_MISSION_GENERATOR_H

#include "map_node.h"
#include "map_graph.h"
#include "dijkstra_solver.h"

#include <list>
#include <vector>

class DriveMissionGenerator {
public:
    DriveMissionGenerator(std::list<MapNode*> optimal_route);

    std::vector<int> get_drive_mission();

private:
    std::vector<int> drive_mission;

};

#endif // DRIVE_MISSION_GENERATOR_H