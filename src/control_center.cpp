#include "control_center.h"

#include <list>
#include <string>

ControlCenter::ControlCenter() {
    
}

void ControlCenter::set_new_map(MapGraph* map_graph) {

}

void ControlCenter::set_position(MapNode* map_node) {

}

void ControlCenter::set_drive_mission(std::list<MapNode*> drive_mission) {

}

void ControlCenter::add_drive_instruction(SemiDriveInstruction semi_drive_instruction) {

}

void ControlCenter::process(int obstacle_distance, int stop_distance) {

}

bool ControlCenter::finished_instruction() {

}

std::string ControlCenter::get_position() {

}

int ControlCenter::get_finished_instruction_id() {
    
}