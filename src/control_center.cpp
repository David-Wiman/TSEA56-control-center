#include "control_center.h"
#include "map_node.h"
#include "log.h"

#include <list>
#include <string>
#include <sstream>

using namespace std;

ControlCenter::ControlCenter(): state{control::stoped_at_node} {
    Logger::log(INFO, __FILE__, "ControlCenter", "Initialize ControlCenter");
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

void ControlCenter::add_drive_instruction(control::Instruction instruction, string id) {
    drive_instruction_t drive_instruction{};
    drive_instruction.instruction = instruction;
    drive_instruction.id = id;
    drive_instructions.push_back(drive_instruction);
}

reference_t ControlCenter::operator()(
        int obstacle_distance, int stop_distance,
        int left_angle, int right_angle, int image_processing_status_code) {
    if (stop_distance == -1)
        stop_distance = 1000;

    stringstream ss;
    ss << "obstacle_distance: " << obstacle_distance
       << ", stop_distance: " << stop_distance;
    Logger::log(DEBUG, __FILE__, "ControlCenter()", ss.str());
    reference_t reference = {0, 0, 1};

    update_state(obstacle_distance, stop_distance);

    // Drive mode
    if ((state == control::running_in_intersection) || image_processing_status_code != 0) {
        reference.drive_mode = 2;
    } else {
        reference.drive_mode = 1;
    }

    reference.angle = calculate_angle(left_angle, right_angle);
    reference.speed = calculate_speed();

    return reference;
}

void ControlCenter::update_state(int obstacle_distance, int stop_distance) {
    drive_instruction_t instr{};

    if (drive_instructions.empty()) {
        // No instruction
        state = control::stoped_at_node;
        return;
    } else {
        instr = drive_instructions.front();
    }

    if (instr.instruction == control::stop) {
        state = control::stoped_at_node;
        finish_instruction();
        return;
    }

    switch (state) {
        case control::running:
        case control::running_in_intersection:
            if (path_blocked(obstacle_distance)) {
                cout << "INFO: path blocked" << endl;
                state = control::stoped_at_obstacle;
            } else if (at_stop_line(stop_distance)) {
                // At node
                cout << "INFO: at node" << endl;
                finish_instruction();
                state = get_new_state();
            } else {
                // Clear path, don't change state
                cout << "DEBUG: clear path" << endl;
            }
            break;

        case control::stoped_at_node:
            if (!path_blocked(obstacle_distance)) {
                // The path isn't blocked
                if (at_stop_line(stop_distance)) {
                    cout << "Error: Still at stop line" << endl;
                } else {
                    // No new stop line close
                    cout << "INFO: Begining next drive mission" << endl;
                    state = control::running;
                }
            } else {
                // The path is blocked
                cout << "INFO: path blocked" << endl;
                state = control::stoped_at_obstacle;
            }
            break;

        case control::stoped_at_obstacle:
            if (!path_blocked(obstacle_distance)) {
                // The path is no longer blocked
                cout << "INFO: Path no longer blocked" << endl;
                state = control::running;
            }
            break;

        default:
            cout << "Error: control center in unknown state" << endl;
    }
}

enum control::ControlState ControlCenter::get_new_state() {
    enum control::Instruction instr{};
    if (drive_instructions.empty()) {
        // No instruction
        return control::stoped_at_node;
    } else {
        instr = drive_instructions.front().instruction;
    }
    switch (instr) {
        case control::forward:
            return control::running;
        case control::left:
        case control::right:
            return control::running_in_intersection;
        case control::stop:
            return control::stoped_at_node;
        default:
            cout << "Error: Unknown state" << endl;
            return control::stoped_at_node;
    }
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
    cout << "Finish instruction" << endl;
    string id = drive_instructions.front().id;
    drive_instructions.pop_front();
    finished_id_buffer.push_back(id);
}

std::string ControlCenter::get_position() {
    return "";
}

int ControlCenter::calculate_speed() {
    switch (state) {
        case control::running:
        case control::running_in_intersection:
            return DEFAULT_SPEED;

        case control::stoped_at_node:
        case control::stoped_at_obstacle:
            return 0;

        default:
            cout << "Error: Unknown state in set_speed" << endl;
            return 0;
    }
}

int ControlCenter::calculate_angle(int left_angle, int right_angle) {
    control::Instruction instr{drive_instructions.front().instruction};
    switch (instr) {
        case control::forward:
            return (left_angle + right_angle) / 2;
        case control::left:
            return left_angle;
        case control::right:
            return right_angle;
        default:
            cout << "Error: Unexpected instruction when calculating angle" << endl;
            return 0;
    }
}

string ControlCenter::get_finished_instruction_id() {
    if (finished_id_buffer.empty()) {
        return "";
    } else {
        string id = finished_id_buffer.front();
        finished_id_buffer.pop_front();
        return id;
    }
}

enum control::ControlState ControlCenter::get_state() {
    return state;
}
