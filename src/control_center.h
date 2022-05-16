#ifndef CONTROLCENTER_H
#define CONTROLCENTER_H

#include "map_node.h"
#include "path_finder.h"
#include "drive_mission_generator.h"
#include "raspi_common.h"
#include "filter.h"
#include "line_detector.h"
#include "constants.h"

#include <string>
#include <list>
#include <vector>

namespace state {
    enum ControlState {normal, intersection, stopping, blocked, stop_line, waiting};
}

class ControlCenter {
public:
    ControlCenter(
            size_t obstacle_distance_filter_len=1,
            size_t stop_distance_filter_len=1,
            int consecutive_param=1,
            int high_count_param=0,
            unsigned status_code_threshold=1);
    void update_map(json m);
    void set_drive_missions(std::list<std::string> target_list);

    void add_drive_instruction(enum instruction::InstructionNumber instr_number, std::string id);
    void add_drive_instruction(drive_instruction_t drive_instruction);

    /* The control center is callable. It must be called every program cycle. */
    control_t operator()(
            int obstacle_distance, int stop_distance, int speed,
            int angle_left, int angle_right, int lateral_left,
            int lateral_right, int image_processing_status_code);

    inline control_t operator()(sensor_data_t sensor_data, image_proc_t image_data) {
        return (*this)(
                sensor_data.obstacle_distance, image_data.stop_distance,
                sensor_data.speed, image_data.angle_left,
                image_data.angle_right, image_data.lateral_left,
                image_data.lateral_right, image_data.status_code
        );
    }

    std::string get_current_road_segment();

    drive_instruction_t get_current_drive_instruction();

    /* Is there a finished instruction whoose id has not yet been extracted? */
    bool finished_instruction();

    /* Return "" if no new instruction have been finished. */
    std::string get_finished_instruction_id();
    enum state::ControlState get_state();

private:
    void update_state(int obstacle_distance, int stop_distence, int speed);

    /* Helpter to calculate new state based on the next instruction. */
    void set_new_state(int speed);

    /* The current instruction is completed. Now:
     * - Remove the current instruction from the buffer
     * - Add it's id to the finished instruction id buffer
     * - Inform path_finder */
    void finish_instruction();

    inline bool path_blocked(int obstacle_distance) {
        return obstacle_distance <= OBST_DISTANCE_CLOSE;
    }

    /* Call after update_state(). */
    int calculate_speed() const;

    /* Set regulation mode in control_data */
    void choose_regulation_mode(control_t *control_data, int image_processing_status_code);

    int calculate_angle(int angle_left, int angle_right);

    inline bool is_expected(int angle) {
        return abs(angle - last_angle) < EXPECTED_ANGLE_THESHOLD;
    }

    int calculate_lateral_position(int lateral_left, int lateral_right) const;

    Filter<int> obstacle_distance_filter;
    Filter<int> stop_distance_filter;
    enum state::ControlState state{state::stop_line};
    enum state::ControlState stop_reason{state::stop_line};
    bool finish_when_stopped{false};
    std::list<drive_instruction_t> drive_instructions{};
    std::list<std::string> finished_id_buffer{};
    unsigned consecutive_0_status_codes{INT_MAX};
    int last_image_status_code{0};
    int last_angle{0};
    std::list<std::string> road_segments{};
    LineDetector stop_line_detector;
    PathFinder path_finder{};
    unsigned status_code_threshold;
};

#endif // CONTROLCENTER_H
