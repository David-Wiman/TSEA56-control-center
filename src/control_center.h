#ifndef CONTROLCENTER_H
#define CONTROLCENTER_H

#include "map_node.h"
#include "path_finder.h"
#include "drive_mission_generator.h"
#include "raspi_common.h"

#include <string>
#include <list>

#define DEFAULT_SPEED 500
#define STOP_DISTANCE_CLOSE 40
#define STOP_DISTANCE_FAR 100
#define OBST_DISTANCE_CLOSE 90

namespace state {
    enum ControlState {running, running_in_intersection, stoped_at_node, stoped_at_obstacle};
}

class ControlCenter {
public:
    ControlCenter();
    //void set_new_map(MapGraph *mapgraph);
    void set_position(std::string current_position_name);
    void update_map(json m);
    std::vector<int> get_drive_instructions(std::string stop_node_name);

    void set_drive_mission(std::list<MapNode*> drive_mission);
    void add_drive_instruction(enum instruction::InstructionNumber instr_number, std::string id);
    void add_drive_instruction(drive_instruction_t drive_instruction);

    /* The control center is callable. It must be called every program cycle. */
    reference_t operator()(
            int obstacle_distance, int stop_distance, int left_angle,
            int right_angle, int image_processing_status_code);

    inline reference_t operator()(sensor_data_t sensor_data, image_proc_t image_proc_data) {
        return (*this)(
                sensor_data.obstacle_distance, image_proc_data.stop_distance,
                image_proc_data.angle_left, image_proc_data.angle_right,
                image_proc_data.status_code
        );
    }

    std::string get_position();
    std::string get_current_road_segment();

    /* Return 0 if no new instruction have been finished. */
    std::string get_finished_instruction_id();
    enum state::ControlState get_state();

private:
    void update_state(int obstacle_distance, int stop_distence);

    /* Helpter to calculate new state based on the next instruction. */
    enum state::ControlState get_new_state();

    /* Remove the current instruction from the buffer, and add it's id to the
     * finished instructin id buffer. */
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

    std::list<int> obstacle_distance_buffer{};
    std::list<int> stop_distance_buffer{};
    enum state::ControlState state;
    std::list<drive_instruction_t> drive_instructions{};
    std::list<std::string> finished_id_buffer{};
    bool have_stoped{false};

    PathFinder path_finder{};
    std::string current_position_name{};
};

#endif // CONTROLCENTER_H
