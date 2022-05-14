#include "catch.hpp"
#include "path_finder.h"
#include "drive_mission_generator.h"
#include "map_node.h"
#include "control_center.h"
#include "log.h"
#include "raspi_common.h"
#include "filter.h"

#include <string>
#include <list>
#include <iostream>
#include <nlohmann/json.hpp>


using namespace std;
using json = nlohmann::json;

TEST_CASE("Map Node") {
    SECTION("Constructors") {
    Logger::init();
        MapNode node1{"1"};
        MapNode node2{"2", 34};
        node2.set_left(2, &node1);
        MapNode node3{"3"};
        node3.set_left(2, &node1);
        node3.set_right(3, &node2);
        Logger::close();
    }
    SECTION("Constructors (pointers)") {
        MapNode *node1 = new MapNode{"1"};
        MapNode *node2 = new MapNode{"2", 34};
        node2->set_left(2, node1);
        MapNode *node3 = new MapNode{"3"};
        node3->set_left(2, node1);
        node3->set_right(3, node2);
        delete node1;
        delete node2;
        delete node3;
    }
    SECTION("Getters and setters") {
        MapNode node1{"1"};
        MapNode node2{"2", 34};
        node2.set_left(2, &node1);
        MapNode node3{"3"};
        node3.set_left(2, &node1);
        node3.set_right(3, &node2);
        CHECK(node1.get_weight() > 100000);
        CHECK(node2.get_weight() == 34);
        node1.set_weight(8);
        CHECK(node1.get_weight() == 8);
    }
    SECTION("Get next") {
        MapNode node1{"1"};
        MapNode node2{"2", 34};
        node2.set_left(2, &node1);
        MapNode node3{"3"};
        node3.set_left(2, &node1);
        node3.set_right(3, &node2);

        CHECK(*node2.get_left().node == node1);
        //CHECK(node2.get_next_right() != node1);
    }
}

TEST_CASE("Path Finder") {
    string map_string = "{\"MapData\": {\"A\": [{\"B\": 3}, {\"C\": 1}], \"B\": [{\"D\": 2}], \"C\": [{\"B\": 1},{\"D\": 5}], \"D\": [] }}";
    SECTION("JSON map") {
        // Create JSON object
        json json_map = json::parse(map_string);
    }
    SECTION("Solver") {
        // Create JSON object
        json json_map = json::parse(map_string);
        // Create a PathFinder, finds best node order, list instructions
        PathFinder finder{json_map, "A"};
    }
    SECTION("Drive Mission") {
        // Create JSON object
        json json_map = json::parse(map_string);
        // Create a PathFinder, finds best node order, vectorize instructions
        PathFinder finder{json_map, "A"};
        vector<instruction::InstructionNumber> drive_mission = finder.get_drive_mission();

        CHECK(drive_mission[0] == instruction::right);
        CHECK(drive_mission[1] == instruction::left);
        CHECK(drive_mission[2] == instruction::forward);
    }
    SECTION("Limited Solve") {
        json json_map = json::parse(map_string);
        PathFinder finder{};
        finder.update_map(json_map);
        finder.solve("A", "B");
        vector<instruction::InstructionNumber> drive_mission = finder.get_drive_mission();
    }
    SECTION("Large Map") {
        string map_string = "{\"MapData\":{\"A1\":[{\"K1\":5}],\"A2\":[{\"B2\":1}],\"B1\":[{\"A1\":1}],\"B2\":[{\"L2\":2},{\"C2\":2}],\"C1\":[{\"B1\":2},{\"L2\":2}],\"C2\":[{\"D2\":1}],\"D1\":[{\"C1\":1}],\"D2\":[{\"E2\":1}],\"E1\":[{\"D1\":1}],\"E2\":[{\"F2\":2}],\"F1\":[{\"E1\":2}],\"F2\":[{\"G2\":3}],\"G1\":[{\"F1\":3}],\"G2\":[{\"H2\":1}],\"H1\":[{\"G1\":1}],\"H2\":[{\"M1\":2},{\"I2\":2}],\"I1\":[{\"H1\":2},{\"M1\":2}],\"I2\":[{\"J2\":1}],\"J1\":[{\"I1\":1}],\"J2\":[{\"K2\":1}],\"K1\":[{\"J1\":1}],\"K2\":[{\"A2\":5}],\"L1\":[{\"C2\":2},{\"B1\":2}],\"L2\":[{\"M2\":1}],\"M1\":[{\"L1\":1}],\"M2\":[{\"I2\":2},{\"H1\":2}]}}";
        json json_map = json::parse(map_string);
        PathFinder finder{};
        finder.update_map(json_map);
        finder.solve("L2", "L1");
        vector<instruction::InstructionNumber> drive_mission = finder.get_drive_mission();
    }
    SECTION("Multiple drive missions on large Map") {
        string map_string = "{\"MapData\":{\"A1\":[{\"K1\":5}],\"A2\":[{\"B2\":1}],\"B1\":[{\"A1\":1}],\"B2\":[{\"L2\":2},{\"C2\":2}],\"C1\":[{\"B1\":2},{\"L2\":2}],\"C2\":[{\"D2\":1}],\"D1\":[{\"C1\":1}],\"D2\":[{\"E2\":1}],\"E1\":[{\"D1\":1}],\"E2\":[{\"F2\":2}],\"F1\":[{\"E1\":2}],\"F2\":[{\"G2\":3}],\"G1\":[{\"F1\":3}],\"G2\":[{\"H2\":1}],\"H1\":[{\"G1\":1}],\"H2\":[{\"M1\":2},{\"I2\":2}],\"I1\":[{\"H1\":2},{\"M1\":2}],\"I2\":[{\"J2\":1}],\"J1\":[{\"I1\":1}],\"J2\":[{\"K2\":1}],\"K1\":[{\"J1\":1}],\"K2\":[{\"A2\":5}],\"L1\":[{\"C2\":2},{\"B1\":2}],\"L2\":[{\"M2\":1}],\"M1\":[{\"L1\":1}],\"M2\":[{\"I2\":2},{\"H1\":2}]}}";
        json json_map = json::parse(map_string);
        PathFinder finder{};
        vector<instruction::InstructionNumber> drive_mission{};
        finder.update_map(json_map);
        finder.solve("L2", "L1");
        drive_mission = finder.get_drive_mission();
        finder.solve("L1", "G1");
        drive_mission = finder.get_drive_mission();
        finder.solve("G1", "J2");
        drive_mission = finder.get_drive_mission();
    }
}

TEST_CASE("Filter") {
    SECTION("Basics") {
        Filter<double> f1{3, 0};
        CHECK( (f1(1.0) - 0.3333) < 0.001 );
        f1(1.0);
        CHECK( (f1(1.0) - 3.0) < 0.001 );

        Filter<int> f2{1, 0};
        CHECK( f2(4) == 4 );
        CHECK( f2(5) == 5 );
    }
}

TEST_CASE("Control Center") {
    SECTION("Basics") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == state::stop_line);
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center(1000, 200, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::normal);
    }
    SECTION("Obstacle detection") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == state::stop_line);

        // Start with obstacle
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center(OBST_DISTANCE_CLOSE-10, 200, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::blocked);
        control_center(OBST_DISTANCE_CLOSE+10, 200, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::normal);

        // Run into obstacle
        control_center(OBST_DISTANCE_CLOSE-10, 200, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::stopping);
        control_center(OBST_DISTANCE_CLOSE-10, 200, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::blocked);
        Logger::close();
    }
    SECTION("Stop-line detection") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == state::stop_line);
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center(OBST_DISTANCE_CLOSE+10, STOP_DISTANCE_FAR, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::normal);

        int stop_distance{-1};
        control_t control_data = control_center(1000, stop_distance, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::normal);
        stop_distance = STOP_DISTANCE_FAR;
        control_data = control_center(1000, stop_distance, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::normal);

        stop_distance = STOP_DISTANCE_MID;
        control_data = control_center(1000, stop_distance, DEFAULT_SPEED, 0, 0, 0, 0, 0);

        stop_distance = STOP_DISTANCE_CLOSE;
        control_data = control_center(1000, stop_distance, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_data.speed_ref == 0);
        CHECK(control_center.get_state() == state::stopping);
        control_data = control_center(1000, stop_distance, 0, 0, 0, 0, 0, 0);
        CHECK(control_data.speed_ref == 0);
        CHECK(control_center.get_state() == state::stop_line);
    }
    SECTION("Drive Instructions") {
        Logger::init();
        ControlCenter control_center{};
        control_t control_data{};
        control_center.add_drive_instruction(instruction::forward, "1");

        control_data = control_center(1000, 600, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");

        control_data = control_center(1000, 500, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        control_data = control_center(1000, 300, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        control_data = control_center(1000, 200, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");

        control_data = control_center(1000, STOP_DISTANCE_MID, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        control_data = control_center(1000, STOP_DISTANCE_CLOSE, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::stopping);
        CHECK(control_center.get_finished_instruction_id() == "");
        control_data = control_center(1000, STOP_DISTANCE_CLOSE, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "1");
        CHECK(control_center.get_finished_instruction_id() == "");
    }
    SECTION("Multiple instructions") {
        Logger::init();
        ControlCenter control_center{};
        control_t control_data{};
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::right, "2");

        // Start at line, ignore it
        control_data = control_center(1000, 10, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::normal);

        // Drive to next line
        control_data = control_center(1000, 200, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::normal);

        // At line, but don't stop. Continue til next line
        control_data = control_center(1000, STOP_DISTANCE_MID, INTERSECTION_SPEED, 0, 0, 0, 0, 0);
        control_data = control_center(1000, STOP_DISTANCE_CLOSE, INTERSECTION_SPEED, 0, 0, 0, 0, 0);

        CHECK(control_center.get_finished_instruction_id() == "1");
        CHECK(control_data.speed_ref == INTERSECTION_SPEED);
        CHECK(control_center.get_state() == state::intersection);

        control_data = control_center(1000, STOP_DISTANCE_CLOSE - 20, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(control_data.speed_ref == INTERSECTION_SPEED);
        CHECK(control_center.get_state() == state::intersection);

        control_data = control_center(1000, STOP_DISTANCE_FAR + 10, INTERSECTION_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(control_data.speed_ref == INTERSECTION_SPEED);
        CHECK(control_center.get_state() == state::intersection);

        // At line, stop
        control_data = control_center(1000, STOP_DISTANCE_MID, INTERSECTION_SPEED, 0, 0, 0, 0, 0);
        control_data = control_center(1000, STOP_DISTANCE_CLOSE - 10, INTERSECTION_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(control_data.speed_ref == 0);
        CHECK(control_center.get_state() == state::stopping);
        control_data = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "2");
        CHECK(control_data.speed_ref == 0);
        CHECK(control_center.get_state() == state::stop_line);

        control_data = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(control_data.speed_ref == 0);
        CHECK(control_center.get_state() == state::stop_line);

        // New instruction
        control_center.add_drive_instruction(instruction::forward, "3");

        // Drive to next line
        control_data = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::normal);

        control_data = control_center(1000, STOP_DISTANCE_FAR + 10, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        CHECK(control_center.get_state() == state::normal);

        // At line, stop
        control_data = control_center(1000, STOP_DISTANCE_MID, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        control_data = control_center(1000, STOP_DISTANCE_CLOSE, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        control_data = control_center(1000, STOP_DISTANCE_CLOSE, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "3");
        CHECK(control_data.speed_ref == 0);
        CHECK(control_center.get_state() == state::stop_line);
    }
    SECTION("Steering control") {
        Logger::init();
        ControlCenter control_center{};
        control_t control_data{};
        int angle_left{};
        int angle_right{};
        int lateral_left{};
        int lateral_right{};

        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::left, "2");
        control_center.add_drive_instruction(instruction::right, "3");

        // Straight road
        angle_left = -1;
        angle_right = 1;
        lateral_left = 3;
        lateral_right = 1;
        control_data = control_center(
                200, 200, 0, angle_left, angle_right,
                lateral_left, lateral_right, 0);
        CHECK(control_data.angle == (angle_left+angle_right)/2);
        CHECK(control_data.lateral_position == (lateral_left+lateral_right)/2);

        // Finish straight part, enter intersection (left turn)
        control_center(200, STOP_DISTANCE_MID, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        control_center(200, STOP_DISTANCE_CLOSE, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::intersection);

        // Left turn
        angle_left = 20;
        angle_right = -300;
        lateral_left = 3;
        lateral_right = 20;
        control_data = control_center(
                200, STOP_DISTANCE_FAR, DEFAULT_SPEED,
                angle_left, angle_right, lateral_left, lateral_right, 0);
        CHECK(control_data.angle == angle_left);
        CHECK(control_data.lateral_position == lateral_left);

        // Finish left turn, enter right turn
        angle_left = 200;
        angle_right = 4;
        lateral_left = 30;
        lateral_right = 1;
        control_data = control_center(
                200, STOP_DISTANCE_MID, DEFAULT_SPEED,
                angle_left, angle_right, lateral_left, lateral_right, 0);
        control_data = control_center(
                200, STOP_DISTANCE_CLOSE, DEFAULT_SPEED,
                angle_left, angle_right, lateral_left, lateral_right, 0);
        CHECK(control_data.angle == angle_right);
        CHECK(control_data.lateral_position == lateral_right);
    }
    SECTION("Drive mode") {
        Logger::init();

        // Everything default, except for status_code_threshold=2
        ControlCenter control_center{1, 1, 1, 0, 2};

        control_t control_data{};
        int image_processing_status_code{0};

        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::left, "2");

        // Straight road
        control_data = control_center(200, 200, 0, 0, 0, 0, 0, image_processing_status_code);
        CHECK(control_data.regulation_mode == regulation_mode::auto_nominal);

        // Bad lateral data from image processing module
        image_processing_status_code = 1;
        control_data = control_center(200, 200, DEFAULT_SPEED, 0, 0, 0, 0, image_processing_status_code);
        CHECK(control_data.regulation_mode == regulation_mode::auto_critical);

        // Good again, but don't change regulation mode until 2 good in a row
        image_processing_status_code = 0;
        control_data = control_center(200, 200, DEFAULT_SPEED, 0, 0, 0, 0, image_processing_status_code);
        CHECK(control_data.regulation_mode == regulation_mode::auto_critical);
        control_data = control_center(200, 200, DEFAULT_SPEED, 0, 0, 0, 0, image_processing_status_code);
        CHECK(control_data.regulation_mode == regulation_mode::auto_nominal);

        // Finish straight part, enter intersection
        control_data = control_center(200, STOP_DISTANCE_FAR, DEFAULT_SPEED, 0, 0, 0, 0, image_processing_status_code);
        control_center(200, STOP_DISTANCE_MID, DEFAULT_SPEED, 0, 0, 0, 0, image_processing_status_code);
        control_center(200, STOP_DISTANCE_CLOSE, DEFAULT_SPEED, 0, 0, 0, 0, image_processing_status_code);
        CHECK(control_center.get_state() == state::intersection);

        // Intersection
        control_data = control_center(200, 200, DEFAULT_SPEED, 0, 0, 0, 0, image_processing_status_code);
        CHECK(control_data.regulation_mode == regulation_mode::auto_nominal);

        image_processing_status_code = 1;
        control_data = control_center(200, 200, DEFAULT_SPEED, 0, 0, 0, 0, image_processing_status_code);
        CHECK(control_data.regulation_mode == regulation_mode::auto_critical);
    }
    SECTION("Simple data types") {
        ControlCenter control_center{};

        sensor_data_t sensor_data{};
        sensor_data.obstacle_distance = OBST_DISTANCE_CLOSE + 10;
        sensor_data.speed = 0;

        image_proc_t image_data{};
        image_data.stop_distance = STOP_DISTANCE_FAR;
        image_data.angle_left = 0;
        image_data.angle_right = 0;
        image_data.status_code = 0;

        control_center.add_drive_instruction(instruction::forward, "1");
        control_t control_data = control_center(sensor_data, image_data);
        CHECK(control_center.get_state() == state::normal);
        CHECK(control_data.regulation_mode == regulation_mode::auto_nominal);
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        CHECK(control_data.angle == 0);

        sensor_data.speed = DEFAULT_SPEED;
        image_data.stop_distance = STOP_DISTANCE_MID;
        control_data = control_center(sensor_data, image_data);
        image_data.stop_distance = STOP_DISTANCE_CLOSE;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_center.get_state() == state::stopping);
        CHECK(control_data.speed_ref == 0);

        sensor_data.speed = 0;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_center.get_state() == state::stop_line);
    }

    SECTION("Robustnes when stopping") {
        ControlCenter control_center{};

        sensor_data_t sensor_data{};
        sensor_data.obstacle_distance = OBST_DISTANCE_CLOSE + 10;
        sensor_data.speed = 0;

        image_proc_t image_data{};
        image_data.stop_distance = STOP_DISTANCE_FAR;
        image_data.angle_left = 0;
        image_data.angle_right = 0;
        image_data.status_code = 0;

        control_center.add_drive_instruction(instruction::forward, "1");
        control_t control_data = control_center(sensor_data, image_data);

        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        sensor_data.speed = DEFAULT_SPEED;

        // Start stopping
        sensor_data.obstacle_distance = OBST_DISTANCE_CLOSE;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.speed_ref == 0);

        // Get incorrect obstacle distance, but don't change state because we
        // havent stopped yet
        sensor_data.obstacle_distance = 10000;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.speed_ref == 0);

        // Finaly stop
        sensor_data.speed = 0;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.speed_ref == 0);
    }

    SECTION("Filter") {
        Logger::init();
        ControlCenter control_center{3, 3};

        sensor_data_t sensor_data{};
        sensor_data.obstacle_distance = 0;
        sensor_data.speed = 0;

        image_proc_t image_data{};
        image_data.stop_distance = STOP_DISTANCE_CLOSE;
        image_data.angle_left = 0;
        image_data.angle_right = 0;
        image_data.status_code = 0;

        control_center.add_drive_instruction(instruction::forward, "1");

        // Start at stop line, ignore it
        control_t control_data = control_center(sensor_data, image_data);
        CHECK(control_center.get_state() == state::normal);
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        sensor_data.speed = DEFAULT_SPEED;

        // Pass the line
        image_data.stop_distance = STOP_DISTANCE_FAR;
        control_data = control_center(sensor_data, image_data);
        control_data = control_center(sensor_data, image_data);
        control_data = control_center(sensor_data, image_data);

        // Don't stop, because we've only seen this once
        sensor_data.obstacle_distance = OBST_DISTANCE_CLOSE - 10;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_center.get_state() == state::normal);
        CHECK(control_data.speed_ref == DEFAULT_SPEED);
        sensor_data.speed = DEFAULT_SPEED;

        control_data = control_center(sensor_data, image_data);
        control_data = control_center(sensor_data, image_data);

        // Now we should be stopping
        CHECK(control_center.get_state() == state::stopping);
        CHECK(control_data.speed_ref == 0);

        // Finaly stop
        sensor_data.speed = 0;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.speed_ref == 0);

    }
    SECTION("Advanced filter") {
        Logger::init();

        ControlCenter control_center{1, 1, 5, 2};
        control_t control_data{};
        int completed_instructions{0};

        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");
        control_center.add_drive_instruction(instruction::forward, "1");

        sensor_data_t sensor_data{};
        sensor_data.obstacle_distance = 0;
        sensor_data.speed = 0;

        image_proc_t image_data{};
        image_data.stop_distance = STOP_DISTANCE_CLOSE;
        image_data.angle_left = 0;
        image_data.angle_right = 0;
        image_data.status_code = 0;

        // Ideal case
        vector<int> stop_distances1{
            20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, -1, -1, -1, -1, -1, -1,
            50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34,
            33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20
        };
        for (int stop_distance : stop_distances1) {
            image_data.stop_distance = stop_distance;
            control_data = control_center(sensor_data, image_data);
            if (control_center.get_finished_instruction_id() == "1")
                completed_instructions++;
        }
        CHECK(completed_instructions == 1);

        completed_instructions = 0;
        vector<int> stop_distances2{
            20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, -1, -1, -1, -1, -1, -1,
            // Bad data between lines
            10, -1, 13, 12, 11, 10, -1, -1, 13, -1, -1, -1, -1, -1, 10, 12, 22,
            -1, -1, 99, 45, 11, -1, -1, -1, 13, -1, -1, -1, -1, -1, 10, 13, 11,
            10, -1, 13, 12, 11, 10, 10, 10, 10, 10, -1, -1, -1, -1, 10, 12, 22,
            // Bad data when detecting lines
            50, 49, -1, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34,
            33, 32, -1, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20
        };

        for (int stop_distance : stop_distances2) {
            image_data.stop_distance = stop_distance;
            control_data = control_center(sensor_data, image_data);
            if (control_center.get_finished_instruction_id() == "1")
                completed_instructions++;
        }
        CHECK(completed_instructions == 1);

        // Real drive data
        vector<int> stop_distances3{86, 86, 86, 86, 87, 86, 88, 87, 86, 86, 86, 86, 86, 86, 85, 86, 86, 86, 86, 86, 87, 86, 86, 86, 87, 86, 86, 87, 85, 86, 86, 86, 87, 88, 87, 85, 86, 85, 86, 85, 86, 86, 86, 86, 86, 86, 87, 85, 86, 86, 88, 86, 87, 85, 87, 86, 86, 86, 86, 85, 86, 86, 85, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 87, 85, 85, 84, 83, 83, 81, 81, 80, 77, 76, 75, 73, 73, 70, 68, 67, 65, 62, 60, 57, 55, 52, 49, 47, 44, 42, 39, 36, 34, 31, 29, 26, 24, 21, 19, 16, 14, 12, 10, 9, -1, -1, -1, -1, -1, -1, -1};
        completed_instructions = 0;
        for (int stop_distance : stop_distances3) {
            image_data.stop_distance = stop_distance;
            control_data = control_center(sensor_data, image_data);
            if (control_center.get_finished_instruction_id() == "1")
                completed_instructions++;
        }
        CHECK(completed_instructions == 1);

        // Very short between lines
        vector<int> stop_distances4{
            71, 70, 69, 67, 66, 65, 63, 61, 60, 58, 56, 54, 52, 49, 47,
            44, 42, 40, 38, 35, 32, 30, 27, 25, 22, 19, 17, 15, 12, 11,
            71, 70, 69, 67, 66, 65, 63, 61, 60, 58, 56, 54, 52, 49, 47,
            44, 42, 40, 38, 35, 32, 30, 27, 25, 22, 19, 17, 15, 12, 11
        };
        completed_instructions = 0;
        for (int stop_distance : stop_distances4) {
            image_data.stop_distance = stop_distance;
            control_data = control_center(sensor_data, image_data);
            if (control_center.get_finished_instruction_id() == "1")
                completed_instructions++;
        }
        CHECK(completed_instructions == 2);
    }

    SECTION("Robust angles") {

        /* Try to prevent the angle from changing unexpectedly. Perhaps from a
         * false detection in the image processor.
         */

        ControlCenter control_center{};
        control_t control_data{};

        sensor_data_t sensor_data{};
        sensor_data.obstacle_distance = 0;
        sensor_data.speed = DEFAULT_SPEED;

        image_proc_t image_data{};
        image_data.stop_distance = STOP_DISTANCE_FAR;
        image_data.angle_left = 0;
        image_data.angle_right = 0;
        image_data.status_code = 0;


        // Normal road. If angles looks good: use both (average),
        // if one looks ok but other does not: use the good one.
        control_center.add_drive_instruction(instruction::forward, "1");
        image_data.angle_left = 10;
        image_data.angle_right = 12;
        control_data = control_center(sensor_data, image_data);
        image_data.angle_left = 22;
        image_data.angle_right = 24;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.angle == 23);

        // Bad left angle
        image_data.angle_left = -22;
        image_data.angle_right = 24;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.angle == 24);

        // Bad left angle, slightly worse right angle. Use right
        image_data.angle_left = -22;
        image_data.angle_right = 12;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.angle == 12);

        // Enter intersection. Drive right
        control_center.add_drive_instruction(instruction::right, "2");
        image_data.stop_distance = STOP_DISTANCE_MID;
        image_data.angle_left = 22;
        image_data.angle_right = 22;
        control_data = control_center(sensor_data, image_data);
        image_data.stop_distance = STOP_DISTANCE_CLOSE;
        control_data = control_center(sensor_data, image_data);

        // Good data, use right angle
        image_data.angle_left = 22;
        image_data.angle_right = 24;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.angle == 24);

        // Bad left data, shuldn't mater
        image_data.angle_left = -22;
        image_data.angle_right = 24;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.angle == 24);

        // Bad right data, but left data is plausable. Use left angle.
        image_data.angle_left = 22;
        image_data.angle_right = -24;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.angle == 22);

        // Both bad. Use right angle.
        image_data.angle_left = -22;
        image_data.angle_right = -24;
        control_data = control_center(sensor_data, image_data);
        CHECK(control_data.angle == -24);
    }

    SECTION("Dijkstra from ControlCenter with list of drive missions") {
        Logger::init();
        // Make map
        string map_string = "{\"MapData\":{\"A1\":[{\"K1\":5}],\"A2\":[{\"B2\":1}],\"B1\":[{\"A1\":1}],\"B2\":[{\"L2\":2},{\"C2\":2}],\"C1\":[{\"B1\":2},{\"L2\":2}],\"C2\":[{\"D2\":1}],\"D1\":[{\"C1\":1}],\"D2\":[{\"E2\":1}],\"E1\":[{\"D1\":1}],\"E2\":[{\"F2\":2}],\"F1\":[{\"E1\":2}],\"F2\":[{\"G2\":3}],\"G1\":[{\"F1\":3}],\"G2\":[{\"H2\":1}],\"H1\":[{\"G1\":1}],\"H2\":[{\"M1\":2},{\"I2\":2}],\"I1\":[{\"H1\":2},{\"M1\":2}],\"I2\":[{\"J2\":1}],\"J1\":[{\"I1\":1}],\"J2\":[{\"K2\":1}],\"K1\":[{\"J1\":1}],\"K2\":[{\"A2\":5}],\"L1\":[{\"C2\":2},{\"B1\":2}],\"L2\":[{\"M2\":1}],\"M1\":[{\"L1\":1}],\"M2\":[{\"I2\":2},{\"H1\":2}]}}";
        json json_map = json::parse(map_string);
        // Make control_center
        ControlCenter control_center{};
        // Prep for solve
        control_center.update_map(json_map);
        control_center.set_drive_missions({"A1", "K2", "H1"});

        CHECK(control_center.get_current_drive_instruction().number == instruction::stop);
        CHECK(!control_center.finished_instruction());
        CHECK(control_center.get_current_road_segment() == "A1");

        control_t control_data{};
        control_data = control_center(OBST_DISTANCE_CLOSE+10, STOP_DISTANCE_CLOSE, 0, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::normal);
        control_data = control_center(OBST_DISTANCE_CLOSE+10, STOP_DISTANCE_FAR, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        CHECK(control_center.get_state() == state::normal);
        CHECK(control_data.speed_ref == DEFAULT_SPEED);

        CHECK(control_center.get_current_drive_instruction().number == instruction::forward);
        CHECK(control_center.finished_instruction());
        CHECK(control_center.get_finished_instruction_id() == "A1");
        CHECK(control_center.get_current_road_segment() == "A1->K1");

        // Drive to next line
        control_data = control_center(OBST_DISTANCE_CLOSE+10, STOP_DISTANCE_MID, DEFAULT_SPEED, 0, 0, 0, 0, 0);
        control_data = control_center(OBST_DISTANCE_CLOSE+10, STOP_DISTANCE_CLOSE, DEFAULT_SPEED, 0, 0, 0, 0, 0);

        CHECK(control_center.get_current_drive_instruction().number == instruction::forward);
        CHECK(control_center.get_current_road_segment() == "K1->J1");
        CHECK(control_center.get_finished_instruction_id() == "A1->K1");
    }
}

