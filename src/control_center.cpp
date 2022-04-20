#include "control_center.h"
#include "map_node.h"

#include <list>
#include <string>

using namespace std;

ControlCenter::ControlCenter()
: obstacle_distance_buffer{}, stop_distance_buffer{},
  state{control::stoped_at_node}, drive_instructions{}, _finished_instruction{false} {
     
}

/*void ControlCenter::set_new_map(MapGraph* map_graph) {

}*/

void ControlCenter::set_position(MapNode* map_node) {

}

void ControlCenter::set_drive_mission(std::list<MapNode*> drive_mission) {

}

void ControlCenter::add_drive_instruction(drive_instruction_t drive_instruction) {
    drive_instructions.push_back(drive_instruction);
}

reference_t ControlCenter::operator()(int obstacle_distance, int stop_distance) {
    reference_t reference = {0, 0};
    switch (state) {
        case control::running:
            break;

        case control::running_in_intersection:
            break;

        case control::stoped_at_node:
            if (drive_instructions.empty()) {
                reference.lateral_position = 0;
                reference.speed = 0;
            } else {
                drive_instruction_t instr = drive_instructions.front();
                if (instr.instruction == control::stop) {
                    _finished_instruction = true;
                    reference.lateral_position = 0;
                    reference.speed = 0;
                } else {
                    state = control::running;
                    reference.lateral_position = 0;
                    reference.speed = DEFAULT_SPEED;
                }
            }
            break;

        case control::stoped_at_obstacle:
            break;

        default: 
            cout << "Error: control center in unknown state" << endl;
    }
    return reference;
}

bool ControlCenter::finished_instruction() {
    return false;
}

std::string ControlCenter::get_position() {
    return "";
}

int ControlCenter::get_finished_instruction_id() {
    return 0;
}

enum control::ControlState ControlCenter::get_state() {
    return state;
}