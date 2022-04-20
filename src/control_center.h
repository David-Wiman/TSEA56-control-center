#ifndef CONTROLCENTER_H
#define CONTROLCENTER_H

#include "map_node.h"

#include <string>
#include <list>

#define DEFAULT_SPEED 10
#define STOP_DISTANCE 30

namespace control {

    enum ControlState {running, running_in_intersection, stoped_at_node, stoped_at_obstacle};
    enum Instruction {left, forward, right, stop};
}

typedef struct DriveInstruction {
    enum control::Instruction instruction;
    int id;
} drive_instruction_t;

typedef struct ReferenceValues {
    int lateral_position;
    int speed;
} reference_t;

class ControlCenter {
public:
    ControlCenter();
    //void set_new_map(MapGraph* mapgraph);
    void set_position(MapNode* mapnode);
    void set_drive_mission(std::list<MapNode*> drive_mission);

    void add_drive_instruction(drive_instruction_t drive_instruction);
    reference_t operator()(int obstacle_distance, int stop_distance);
    bool finished_instruction();

    std::string get_position();
    int get_finished_instruction_id();
    enum control::ControlState get_state();

private:
    //MapGraph map;
    std::list<int> obstacle_distance_buffer;
    std::list<int> stop_distance_buffer;
    enum control::ControlState state;
    std::list<drive_instruction_t> drive_instructions;
    bool _finished_instruction;
};

#endif // CONTROLCENTER_H