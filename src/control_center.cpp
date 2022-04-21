#include "control_center.h"
#include "map_node.h"

#include <list>
#include <string>

using namespace std;

ControlCenter::ControlCenter(): state{control::stoped_at_node} {}

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
    if (stop_distance == -1)
        stop_distance = 1000;

    cout << "Int stop_distance: " << stop_distance << endl;
    reference_t reference = {0, 0};
    drive_instruction_t instr{};

    if (drive_instructions.empty()) {
        // No instruction
        return reference;
    } else {
        instr = drive_instructions.front();
    }
    if (instr.instruction == control::stop) {
        state = control::stoped_at_node;
        finish_instruction();
        reference.speed = 0;
    }

    switch (state) {
        case control::running:
            if (obstacle_distance <= STOP_DISTANCE_CLOSE) {
                // The path is blocked
                cout << "path blocked" << endl;
                state = control::stoped_at_obstacle;
                reference.lateral_position = 0;
                reference.speed = 0;
            } else if (at_stop_line(stop_distance)) {
                // At node
                cout << "at node" << endl;
                finish_instruction();
                if (drive_instructions.empty()) {
                    // Stop
                    state = control::stoped_at_node;
                    reference.lateral_position = 0;
                    reference.speed = 0;
                } else {
                    // More instructions, continue driving
                    reference.lateral_position = 0;
                    reference.speed = DEFAULT_SPEED;
                }
            } else {
                // Clear path
                cout << "clear path" << endl;
                reference.lateral_position = 0;
                reference.speed = DEFAULT_SPEED;
            }
            break;

        case control::running_in_intersection:
            // TODO
            break;

        case control::stoped_at_node:
            if (obstacle_distance > STOP_DISTANCE_CLOSE) {
                // The path isn't blocked
                if (at_stop_line(stop_distance)) {
                    cout << "Error: Still at stop line" << endl;
                    reference.lateral_position = 0;
                    reference.speed = 0;
                } else {
                    // No new stop line close
                    state = control::running;
                    reference.lateral_position = 0;
                    reference.speed = DEFAULT_SPEED;
                }
            } else {
                // The path is blocked
                state = control::stoped_at_obstacle;
                reference.lateral_position = 0;
                reference.speed = 0;
            }
            break;

        case control::stoped_at_obstacle:
            if (obstacle_distance > STOP_DISTANCE_CLOSE) {
                // The path is no longer blocked
                cout << "INFO: Path no longer blocked" << endl;
                state = control::running;
                reference.lateral_position = 0;
                reference.speed = DEFAULT_SPEED;
            } else {
                // The path is still blocked
                reference.lateral_position = 0;
                reference.speed = 0;
            }
            break;

        default: 
            cout << "Error: control center in unknown state" << endl;
    }
    return reference;
}

bool ControlCenter::at_stop_line(int stop_distance) {
    if (stop_distance >= STOP_DISTANCE_FAR) {
        // Next stop line is very far away
        have_stoped = false;
        return false;
    }
    if (stop_distance <= STOP_DISTANCE_CLOSE) {
        if (have_stoped) {
            return false;
        } else {
            // We have not stoped at this line. Stop!
            have_stoped = true;
            return true;
        }
    } else {
        // Stop distance is not close enough to stop
        return false;
    }
}

void ControlCenter::finish_instruction() {
    int id = drive_instructions.front().id;
    drive_instructions.pop_front();
    finished_id_buffer.push_back(id);
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
