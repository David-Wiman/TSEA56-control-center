#include "control_center.h"
#include "map_node.h"
#include "log.h"

#include <list>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

ControlCenter::ControlCenter() {
    Logger::log(INFO, __FILE__, "ControlCenter", "Initialize ControlCenter");
}

void ControlCenter::set_position(MapNode *map_node) {

}

void ControlCenter::set_drive_mission(std::list<MapNode*> drive_mission) {

}

void ControlCenter::add_drive_instruction(drive_instruction_t drive_instruction) {
    drive_instructions.push_back(drive_instruction);
}

void ControlCenter::add_drive_instruction(instruction::InstructionNumber instruction, string id) {
    drive_instruction_t drive_instruction{};
    drive_instruction.number = instruction;
    drive_instruction.id = id;
    drive_instructions.push_back(drive_instruction);
}

reference_t ControlCenter::operator()(
        int obstacle_distance, int stop_distance, int speed,
        int left_angle, int right_angle, int image_processing_status_code) {
    stringstream ss;
    ss << "obstacle_distance: " << obstacle_distance
       << ", stop_distance: " << stop_distance;
    Logger::log(DEBUG, __FILE__, "ControlCenter()", ss.str());
    reference_t reference = {0, 0, regulation_mode::auto_nominal};

    if (stop_distance == -1)
        stop_distance = 1000;

    if (obstacle_distance == 0)
        obstacle_distance = 1000;

    update_state(obstacle_distance, stop_distance, speed);

    // Drive mode
    if ((state == state::intersection) || image_processing_status_code != 0) {
        reference.regulation_mode = regulation_mode::auto_critical;
    } else {
        reference.regulation_mode = regulation_mode::auto_nominal;
    }

    reference.angle = calculate_angle(left_angle, right_angle);
    reference.speed = calculate_speed();

    ss.str("");
    ss << "angle = " << reference.angle
       << ", speed = " << reference.speed
       << ", drive mode = " << reference.regulation_mode;
    Logger::log(DEBUG, __FILE__, "reference", ss.str());

    return reference;
}

void ControlCenter::update_state(int obstacle_distance, int stop_distance, int speed) {
    drive_instruction_t intr{};

    if (drive_instructions.empty()) {
        // No instruction
        if (state != state::stop_line) {
            Logger::log(ERROR, __FILE__, "Update state", "No instruction but state not stop_line");
        }
        if (speed > 0) {
            state = state::stopping;
            stop_reason = state::stop_line;
        } else {
            state = state::stop_line;
        }
        return;
    } else {
        intr = drive_instructions.front();
    }

    //if (intr.number == instruction::stop) {
        //if (speed > 0) {
            //state = state::stopping;
            //stop_reason = state::stop_line;
        //} else {
            //state = state::stop_line;
            //finish_instruction();
        //}
        //return;
    //}

    switch (state) {
        case state::normal:
        case state::intersection:
            if (path_blocked(obstacle_distance)) {
                Logger::log(INFO, __FILE__, "Update state", "Path blocked, stopping");
                state = state::stopping;
                stop_reason = state::blocked;
            } else if (at_stop_line(stop_distance)) {
                // At node
                if (drive_instructions.size() > 1) {
                    finish_instruction();
                    set_new_state(speed);
                } else {
                    Logger::log(INFO, __FILE__, "Update state", "At stop line, stopping");
                    finish_when_stopped = true;
                    state = state::stopping;
                    stop_reason = state::stop_line;
                }
            } else {
                // Clear path, don't change state
                Logger::log(DEBUG, __FILE__, "Update state", "Running");
            }
            break;

        case state::stop_line:
            if (path_blocked(obstacle_distance)) {
                state = state::blocked;
                Logger::log(INFO, __FILE__, "Update state", "Path blocked");
                break;
            }
            if (at_stop_line(stop_distance)) {
                Logger::log(ERROR, __FILE__, "Update state", "Still at stop line");
            }
            set_new_state(speed);
            break;

        case state::blocked:
            if (!path_blocked(obstacle_distance)) {
                // The path is no longer blocked
                Logger::log(INFO, __FILE__, "Update state", "Path no longer blocked");
                set_new_state(speed);
            }
            break;

        case state::stopping:
            if (speed == 0) {
                Logger::log(INFO, __FILE__, "Update state", "Stopped");
                state = stop_reason;
                if (finish_when_stopped) {
                    finish_instruction();
                    finish_when_stopped = false;
                }
            }
            break;

        default:
            Logger::log(ERROR, __FILE__, "Update state", "Unknown state");
    }
}

void ControlCenter::set_new_state(int speed) {
    enum instruction::InstructionNumber instr{};
    enum state::ControlState new_state{};
    string state_name{};
    if (drive_instructions.empty()) {
        // No instruction
        instr = instruction::stop;
    } else {
        instr = drive_instructions.front().number;
    }
    switch (instr) {
        case instruction::forward:
            new_state = state::normal;
            state_name = "normal";
            break;
        case instruction::left:
        case instruction::right:
            new_state = state::intersection;
            state_name = "intersection";
            break;
        case instruction::stop:
            if (speed > 0) {
                stop_reason = state::stop_line;
                new_state = state::stopping;
                state_name = "stopping";
            } else {
                new_state = state::stop_line;
            }
            break;
        default:
            Logger::log(ERROR, __FILE__, "Set new state", "Unknown instruction");
            new_state = state::stop_line;
            state_name = "stop_line";
            break;
    }
    if (new_state != state) {
        Logger::log(INFO, __FILE__, "Set new state", state_name);
        state = new_state;
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
    Logger::log(INFO, __FILE__, "ControlCenter", "Finishing instruction");
    string id = drive_instructions.front().id;
    drive_instructions.pop_front();
    finished_id_buffer.push_back(id);
    path_finder.done_with_drive_instruction();
}

string ControlCenter::get_current_road_segment() {
    return path_finder.get_current_road_segment();
}

int ControlCenter::calculate_speed() {
    switch (state) {
        case state::normal:
        case state::intersection:
            return DEFAULT_SPEED;

        case state::stop_line:
        case state::blocked:
            return 0;

        default:
            Logger::log(ERROR, __FILE__, "ControlCenter", "Unknown state while calculating speed");
            return 0;
    }
}

int ControlCenter::calculate_angle(int left_angle, int right_angle) {
    instruction::InstructionNumber instr{drive_instructions.front().number};
    switch (instr) {
        case instruction::forward:
            return (left_angle + right_angle) / 2;
        case instruction::left:
            return left_angle;
        case instruction::right:
            return right_angle;
        default:
            Logger::log(ERROR, __FILE__, "ControlCenter", "Unknown state while calculating angle");
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

enum state::ControlState ControlCenter::get_state() {
    return state;
}
