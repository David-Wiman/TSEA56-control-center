#include "catch.hpp"
#include "map_node.h"
#include "control_center.h"
#include "log.h"
#include "raspi_common.h"

#include <iostream>

using namespace std;

TEST_CASE("Map Node") {
    SECTION("Constructors") {
        Logger::init();
        MapNode node1{"1", nullptr, nullptr};
        MapNode node2{"2", &node1, nullptr, 34};
        MapNode node3{"3", &node1, &node2};
        Logger::close();
    }
    SECTION("Constructors (pointers)") {
        MapNode *node1 = new MapNode{"1", nullptr, nullptr};
        MapNode *node2 = new MapNode{"2", node1, nullptr, 34};
        MapNode *node3 = new MapNode{"3", node1, node2};
        delete node1;
        delete node2;
        delete node3;
    }
    SECTION("Getters and setters") {
        MapNode node1{"1", nullptr, nullptr};
        MapNode node2{"2", &node1, nullptr, 34};
        MapNode node3{"3", &node1, &node2};
        CHECK(node1.get_weight() > 100000);
        CHECK(node2.get_weight() == 34);
        node1.set_weight(8);
        CHECK(node1.get_weight() == 8);
    }
    SECTION("Get next") {
        MapNode node1{"1", nullptr, nullptr};
        MapNode node2{"2", &node1, nullptr, 34};
        MapNode node3{"3", &node1, &node2};

        CHECK(*node2.get_next_left() == node1);
        //CHECK(node2.get_next_right() != node1);
    }
}

TEST_CASE("Control Center") {
    SECTION("Basics") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == state::stoped_at_node);
        control_center.add_drive_instruction(instruction::left, "1");
        control_center(1000, 200, 0, 0, 0);
        CHECK(control_center.get_state() == state::running);
    }
    SECTION("Obstacle detection") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == state::stoped_at_node);
        control_center.add_drive_instruction(instruction::left, "1");
        control_center(STOP_DISTANCE_CLOSE-10, 200, 0, 0, 0);
        CHECK(control_center.get_state() == state::stoped_at_obstacle);
        control_center(STOP_DISTANCE_CLOSE+10, 200, 0, 0, 0);
        CHECK(control_center.get_state() == state::running);

        vector<int> obstacle_distances{1000, 1000, 100, 75, 60, 55, 30, 25, 15, 10, 2, 1000};
        for (auto distance : obstacle_distances) {
            reference_t ref = control_center(distance, 200, 0, 0, 0);
            if (distance > STOP_DISTANCE_CLOSE) {
                CHECK(ref.speed == DEFAULT_SPEED);
                CHECK(control_center.get_state() == state::running);
            } else {
                CHECK(ref.speed == 0);
                CHECK(control_center.get_state() == state::stoped_at_obstacle);
            }
        }
    Logger::close();
    }
    SECTION("Stop-line detection") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == state::stoped_at_node);
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center(STOP_DISTANCE_CLOSE-10, 200, 0, 0, 0);
        CHECK(control_center.get_state() == state::stoped_at_obstacle);
        control_center(STOP_DISTANCE_CLOSE+10, 200, 0, 0, 0);
        CHECK(control_center.get_state() == state::running);

        int distance{-1};
        reference_t ref = control_center(1000, distance, 0, 0, 0);
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::running);
        distance = 100;
        ref = control_center(1000, distance, 0, 0, 0);
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::running);

        distance = STOP_DISTANCE_CLOSE;
        ref = control_center(1000, distance, 0, 0, 0);
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == state::stoped_at_node);
    }
    SECTION("Drive Instructions") {
        Logger::init();
        ControlCenter control_center{};
        reference_t ref{};
        control_center.add_drive_instruction(instruction::forward, "1");

        ref = control_center(1000, 600, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");

        ref = control_center(1000, 500, 0, 0, 0);
        ref = control_center(1000, 300, 0, 0, 0);
        ref = control_center(1000, 200, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");

        ref = control_center(1000, STOP_DISTANCE_CLOSE, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "1");
        CHECK(control_center.get_finished_instruction_id() == "");
    }
    SECTION("Multiple instructions") {
        Logger::init();
        ControlCenter control_center{};
        reference_t ref{};
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::right, "2");

        // Drive to next line
        ref = control_center(1000, 200, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::running);

        // At line, but don't stop. Continue til next line
        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "1");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::running_in_intersection);

        ref = control_center(1000, STOP_DISTANCE_CLOSE - 20, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::running_in_intersection);

        ref = control_center(1000, STOP_DISTANCE_FAR + 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::running_in_intersection);

        // At line, stop
        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "2");
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == state::stoped_at_node);

        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == state::stoped_at_node);

        // New instruction
        control_center.add_drive_instruction(instruction::left, "3");

        // Drive to next line
        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::running);

        ref = control_center(1000, STOP_DISTANCE_FAR + 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::running);

        // At line, stop
        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "3");
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == state::stoped_at_node);
    }
    SECTION("Intersections") {
        Logger::init();
        ControlCenter control_center{};
        reference_t ref{};
        int left_angle{};
        int right_angle{};

        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::left, "2");

        // Straight road
        left_angle = -1;
        right_angle = 1;
        ref = control_center(200, 200, left_angle, right_angle, 0);
        CHECK(ref.angle == (left_angle+right_angle)/2);

        // Finish straight part, enter intersection
        control_center(200, STOP_DISTANCE_CLOSE, 0, 0, 0);
        CHECK(control_center.get_state() == state::running_in_intersection);

        // Left turn
        left_angle = 20;
        right_angle = -300;
        ref = control_center(200, 200, left_angle, right_angle, 0);
        CHECK(ref.angle == left_angle);
    }
    SECTION("Drive mode") {
        Logger::init();
        ControlCenter control_center{};
        reference_t ref{};
        int image_processing_status_code{0};

        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::left, "2");

        // Straight road
        ref = control_center(200, 200, 0, 0, image_processing_status_code);
        CHECK(ref.regulation_mode == regulation_mode::auto_nominal);

        // Bad lateral data from image processing module
        image_processing_status_code = 1;
        ref = control_center(200, 200, 0, 0, image_processing_status_code);
        CHECK(ref.regulation_mode == regulation_mode::auto_critical);


        // Finish straight part, enter intersection
        image_processing_status_code = 0;
        ref = control_center(200, 200, 0, 0, image_processing_status_code);
        control_center(200, STOP_DISTANCE_CLOSE, 0, 0, image_processing_status_code);
        CHECK(control_center.get_state() == state::running_in_intersection);

        // Intersection
        ref = control_center(200, 200, 0, 0, image_processing_status_code);
        CHECK(ref.regulation_mode == regulation_mode::auto_critical);
    }
    SECTION("image_proc_t") {
        ControlCenter control_center{};

        sensor_data_t sensor_data{};
        sensor_data.obstacle_distance = STOP_DISTANCE_FAR + 10;

        image_proc_t image_data{};
        image_data.stop_distance = STOP_DISTANCE_FAR + 10;
        image_data.angle_left = 0;
        image_data.angle_right = 0;
        image_data.status_code = 0;

        control_center.add_drive_instruction(instruction::forward, "1");
        reference_t ref = control_center(sensor_data, image_data);
        CHECK(control_center.get_state() == state::running);
        CHECK(ref.regulation_mode == regulation_mode::auto_nominal);
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(ref.angle == 0);
    }
}
