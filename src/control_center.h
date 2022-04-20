#ifndef CONTROLCENTER_H
#define CONTROLCENTER_H

#include "map_node.h"

#include <string>
#include <list>

enum ControlState {running, running_in_intersection, stoped_at_node, stoped_at_obstacle};
enum Instruction {left, forward, right, stop};

struct DriveInstruction {
    enum Instruction instruction;
    int id;
} drive_intstruction_t

class ControlCenter {
public:
    ControlCenter();
    //void set_new_map(MapGraph* mapgraph);
    void set_position(MapNode* mapnode);
    void set_drive_mission(std::list<MapNode*> drive_mission);

    //void add_drive_instruction(SemiDriveInstruction semidriveinstruction);
    void process(int obstacle_distance, int stop_distance);
    bool finished_instruction();

    std::string get_position();
    int get_finished_instruction_id();

private:
    //MapGraph map;
    std::list<int> obstacle_distance_buffer;
    std::list<int> stop_distance_buffer;
    enum ControlState state;
    int finished_instruction_id = -1;
};

#endif // CONTROLCENTER_H