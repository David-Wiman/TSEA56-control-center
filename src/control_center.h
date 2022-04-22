#ifndef CONTROLCENTER_H
#define CONTROLCENTER_H

#include "map_node.h"

#include <string>
#include <list>

#define DEFAULT_SPEED 10
#define STOP_DISTANCE_CLOSE 30
#define STOP_DISTANCE_FAR 100

namespace control {

    enum ControlState {running, running_in_intersection, stoped_at_node, stoped_at_obstacle};
    enum Instruction {left, forward, right, stop};
}

typedef struct DriveInstruction {
    enum control::Instruction instruction;
    int id;
} drive_instruction_t;

typedef struct ReferenceValues {
    int speed;
    int angle;
    int drive_mode;
} reference_t;

class ControlCenter {
public:
    ControlCenter();
    //void set_new_map(MapGraph* mapgraph);
    void set_position(MapNode* mapnode);
    void set_drive_mission(std::list<MapNode*> drive_mission);

    void add_drive_instruction(drive_instruction_t drive_instruction);

    /* The control center is callable. It must be called every program cycle. */
    reference_t operator()(
            int obstacle_distance, int stop_distance, int left_angle, 
            int right_angle, int image_processing_status_code);

    std::string get_position();

    /* Return 0 if no new instruction have been finished. */
    int get_finished_instruction_id();
    enum control::ControlState get_state();

private:
    void finish_instruction();

    /* Return true if the car is at a stop line (which it have not been at 
     * before), otherwise return false. */
    bool at_stop_line(int stop_distance);

    //MapGraph map;
    std::list<int> obstacle_distance_buffer{};
    std::list<int> stop_distance_buffer{};
    enum control::ControlState state;
    std::list<drive_instruction_t> drive_instructions{};
    std::list<int> finished_id_buffer{};
    bool have_stoped{false};
};

#endif // CONTROLCENTER_H
