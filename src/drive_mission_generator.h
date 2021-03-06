#ifndef DRIVE_MISSION_GENERATOR_H
#define DRIVE_MISSION_GENERATOR_H

#include "raspi_common.h"
#include "map_node.h"
#include "path_finder.h"

#include <list>
#include <vector>

class DriveMissionGenerator {
public:
    DriveMissionGenerator(std::list<MapNode*> optimal_route);
    DriveMissionGenerator(std::vector<MapNode*> nodes_vector, std::string stop_node_name);

    std::vector<instruction::InstructionNumber> get_drive_mission();

private:
    std::vector<instruction::InstructionNumber> drive_mission;
    std::vector<MapNode*> drive_mission_nodes;

};

#endif // DRIVE_MISSION_GENERATOR_H
