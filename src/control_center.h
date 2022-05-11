#ifndef CONTROLCENTER_H
#define CONTROLCENTER_H

#include "map_node.h"
#include "path_finder.h"
#include "drive_mission_generator.h"
#include "raspi_common.h"
#include "filter.h"

#include <string>
#include <list>
#include <vector>

#define DEFAULT_SPEED 500
#define INTERSECTION_SPEED 400
#define STOP_DISTANCE_CLOSE 30
#define STOP_DISTANCE_MID 50
#define STOP_DISTANCE_FAR 70
#define OBST_DISTANCE_CLOSE 90

namespace state {
    enum ControlState {normal, intersection, stopping, blocked, stop_line, waiting};
}

namespace stop_line {
    enum StopLine {close, mid, far};
}

class ControlCenter {
public:
    ControlCenter(
            size_t obstacle_distance_filter_len=1,
            size_t stop_distance_filter_len=1,
            int consecutive_param=1,
            int high_count_param=0);
    //void set_new_map(MapGraph *mapgraph);
    void set_position(std::string current_position_name);
    void update_map(json m);
    std::vector<int> get_drive_instructions(std::string stop_node_name);
    void finish_drive_mission();
    void update_list_of_target_nodes(std::list<std::string> target_node_name_list);
    void update_list_of_target_nodes_with_DriveMission(DriveMission dm);
    std::vector<int> get_drive_instructions_to_next_target_node();

    void add_drive_instruction(enum instruction::InstructionNumber instr_number, std::string id);
    void add_drive_instruction(drive_instruction_t drive_instruction);

    /* The control center is callable. It must be called every program cycle. */
    reference_t operator()(
            int obstacle_distance, int stop_distance, int speed,
            int left_angle, int right_angle, int image_processing_status_code);

    inline reference_t operator()(sensor_data_t sensor_data, image_proc_t image_proc_data) {
        return (*this)(
                sensor_data.obstacle_distance, image_proc_data.stop_distance,
                sensor_data.speed, image_proc_data.angle_left,
                image_proc_data.angle_right, image_proc_data.status_code
        );
    }

    std::string get_current_road_segment();
    std::string get_current_road_segment_as_json();

    /* Return 0 if no new instruction have been finished. */
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

    /* Return true if the car is at a stop line (which it have not been at
     * before), otherwise return false.
     *
     * Note, this method must be called exactly once per program cycle. */
    bool at_stop_line(int stop_distance);

    /* Call after update_state(). */
    int calculate_speed();

    /* Call after update_state(). */
    int calculate_angle(int left_angle, int right_angle);

    Filter<int> obstacle_distance_filter;
    Filter<int> stop_distance_filter;
    enum state::ControlState state{state::stop_line};
    enum state::ControlState stop_reason{state::stop_line};
    bool finish_when_stopped{false};
    std::list<drive_instruction_t> drive_instructions{};
    std::list<std::string> finished_id_buffer{};
    //bool have_stoped{true};
    int last_stop_distance{100};
    int far_stop_counter{0};
    int consecutive_decreasing_stop_distances{0};
    enum stop_line::StopLine stop_line_mode{stop_line::close};
    int consecutive_param;
    int high_count_param;
    std::list<std::string> road_segments{};

    PathFinder path_finder{};
    std::string current_position_name{};
    std::string current_target_name{};
    std::list<std::string> target_node_name_list{};
};

#endif // CONTROLCENTER_H
