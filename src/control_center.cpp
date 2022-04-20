#include "control_center.h"
#include "map_node.h"

#include <list>
#include <string>

ControlCenter::ControlCenter()
: obstacle_distance_buffer{}, stop_distance_buffer{}, state{instructions::stoped_at_node}  {
    
}

/*void ControlCenter::set_new_map(MapGraph* map_graph) {

}*/

void ControlCenter::set_position(MapNode* map_node) {

}

void ControlCenter::set_drive_mission(std::list<MapNode*> drive_mission) {

}

void ControlCenter::add_drive_instruction(drive_intstruction_t drive_instruction) {

}

reference_t ControlCenter::operator()(int obstacle_distance, int stop_distance) {
    return {0,0};
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

enum instructions::ControlState ControlCenter::get_state() {
    return state;
}