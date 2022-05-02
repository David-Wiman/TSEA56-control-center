#include "catch.hpp"
#include "path_finder.h"
#include "drive_mission_generator.h"
#include "map_node.h"
#include "control_center.h"
#include "log.h"
#include "raspi_common.h"

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
    string map_string = "{\"MapData\": {\"A\": {\"B\": 3, \"C\": 1}, \"B\": {\"D\": 2}, \"C\": {\"B\": 1, \"D\": 5}, \"D\": {} }}";
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
        vector<int> drive_mission = finder.get_drive_mission();

        CHECK(drive_mission[0] == 2);
        CHECK(drive_mission[1] == 0);
        CHECK(drive_mission[2] == 1);
    }
    SECTION("Get current drive mission") {
        Logger::init();
        // Create JSON object
        json json_map = json::parse(map_string);
        // Create a PathFinder, finds best node order, vectorize instructions
        PathFinder finder{json_map, "A"};
        vector<int> drive_mission = finder.get_drive_mission();

        CHECK(finder.get_current_drive_instruction() == 2);
        finder.done_with_drive_instruction();
        CHECK(finder.get_current_drive_instruction() == 0);
        finder.done_with_drive_instruction();
        CHECK(finder.get_current_drive_instruction() == 1);
        Logger::close();
    }
    // SECTION("Limited Solve") {
    //     json json_map = json::parse(map_string);
    //     PathFinder finder{};
    //     finder.update_map(json_map);
    //     finder.solve("A", "B");
    //     vector<int> drive_mission = finder.get_drive_mission();
    // }
    SECTION("Large Map") {
        string map_string = "{\"MapData\":{\"A1\":{\"K1\":5},\"A2\":{\"B2\":1},\"B1\":{\"A1\":1},\"B2\":{\"L2\":2,\"C2\":2},\"C1\":{\"B1\":2,\"L2\":2},\"C2\":{\"D2\":1},\"D1\":{\"C1\":1},\"D2\":{\"E2\":1},\"E1\":{\"D1\":1},\"E2\":{\"F2\":2},\"F1\":{\"E1\":2},\"F2\":{\"G2\":3},\"G1\":{\"F1\":3},\"G2\":{\"H2\":1},\"H1\":{\"G1\":1},\"H2\":{\"M1\":2,\"I2\":2},\"I1\":{\"H1\":2,\"M1\":2},\"I2\":{\"J2\":1},\"J1\":{\"I1\":1},\"J2\":{\"K2\":1},\"K1\":{\"J1\":1},\"K2\":{\"A2\":5},\"L1\":{\"C2\":2,\"B1\":2},\"L2\":{\"M2\":1},\"M1\":{\"L1\":1},\"M2\":{\"I2\":2,\"H1\":2}}}";
        json json_map = json::parse(map_string);  
        PathFinder finder{};
        finder.update_map(json_map);
        finder.solve("A1", "G1");
        vector<int> drive_mission = finder.get_drive_mission();
        cout << "  ";
        for (auto n : drive_mission) {
            cout << n << "  ";
        }
        cout << endl;
    }
}

TEST_CASE("Control Center") {
    SECTION("Basics") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == control::stoped_at_node);
        control_center.add_drive_instruction(control::left, "1");
        control_center(1000, 200, 0, 0, 0);
        CHECK(control_center.get_state() == control::running);
    }
    SECTION("Obstacle detection") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == control::stoped_at_node);
        control_center.add_drive_instruction(control::left, "1");
        control_center(STOP_DISTANCE_CLOSE-10, 200, 0, 0, 0);
        CHECK(control_center.get_state() == control::stoped_at_obstacle);
        control_center(STOP_DISTANCE_CLOSE+10, 200, 0, 0, 0);
        CHECK(control_center.get_state() == control::running);

        vector<int> obstacle_distances{1000, 1000, 100, 75, 60, 55, 30, 25, 15, 10, 2, 1000};
        for (auto distance : obstacle_distances) {
            reference_t ref = control_center(distance, 200, 0, 0, 0);
            if (distance > STOP_DISTANCE_CLOSE) {
                CHECK(ref.speed == DEFAULT_SPEED);
                CHECK(control_center.get_state() == control::running);
            } else {
                CHECK(ref.speed == 0);
                CHECK(control_center.get_state() == control::stoped_at_obstacle);
            }
        }
    Logger::close();
    }
    SECTION("Stop-line detection") {
        Logger::init();
        ControlCenter control_center{};
        CHECK(control_center.get_state() == control::stoped_at_node);
        control_center.add_drive_instruction(control::forward, "1");
        control_center(STOP_DISTANCE_CLOSE-10, 200, 0, 0, 0);
        CHECK(control_center.get_state() == control::stoped_at_obstacle);
        control_center(STOP_DISTANCE_CLOSE+10, 200, 0, 0, 0);
        CHECK(control_center.get_state() == control::running);

        int distance{-1};
        reference_t ref = control_center(1000, distance, 0, 0, 0);
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running);
        distance = 100;
        ref = control_center(1000, distance, 0, 0, 0);
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running);

        distance = STOP_DISTANCE_CLOSE;
        ref = control_center(1000, distance, 0, 0, 0);
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == control::stoped_at_node);
    }
    SECTION("Drive Instructions") {
        Logger::init();
        ControlCenter control_center{};
        reference_t ref{};
        control_center.add_drive_instruction(control::forward, "1");

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
        control_center.add_drive_instruction(control::forward, "1");
        control_center.add_drive_instruction(control::right, "2");

        // Drive to next line
        ref = control_center(1000, 200, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running);

        // At line, but don't stop. Continue til next line
        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "1");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running_in_intersection);

        ref = control_center(1000, STOP_DISTANCE_CLOSE - 20, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running_in_intersection);

        ref = control_center(1000, STOP_DISTANCE_FAR + 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running_in_intersection);

        // At line, stop
        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "2");
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == control::stoped_at_node);

        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == control::stoped_at_node);

        // New instruction
        control_center.add_drive_instruction(control::left, "3");

        // Drive to next line
        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running);

        ref = control_center(1000, STOP_DISTANCE_FAR + 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "");
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running);

        // At line, stop
        ref = control_center(1000, STOP_DISTANCE_CLOSE - 10, 0, 0, 0);
        CHECK(control_center.get_finished_instruction_id() == "3");
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == control::stoped_at_node);
    }
    SECTION("Intersections") {
        Logger::init();
        ControlCenter control_center{};
        reference_t ref{};
        int left_angle{};
        int right_angle{};

        control_center.add_drive_instruction(control::forward, "1");
        control_center.add_drive_instruction(control::left, "2");

        // Straight road
        left_angle = -1;
        right_angle = 1;
        ref = control_center(200, 200, left_angle, right_angle, 0);
        CHECK(ref.angle == (left_angle+right_angle)/2);

        // Finish straight part, enter intersection
        control_center(200, STOP_DISTANCE_CLOSE, 0, 0, 0);
        CHECK(control_center.get_state() == control::running_in_intersection);

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

        control_center.add_drive_instruction(control::forward, "1");
        control_center.add_drive_instruction(control::left, "2");

        // Straight road
        ref = control_center(200, 200, 0, 0, image_processing_status_code);
        CHECK(ref.drive_mode == drive_mode::auto_nominal);

        // Bad lateral data from image processing module
        image_processing_status_code = 1;
        ref = control_center(200, 200, 0, 0, image_processing_status_code);
        CHECK(ref.drive_mode == drive_mode::auto_critical);


        // Finish straight part, enter intersection
        image_processing_status_code = 0;
        ref = control_center(200, 200, 0, 0, image_processing_status_code);
        control_center(200, STOP_DISTANCE_CLOSE, 0, 0, image_processing_status_code);
        CHECK(control_center.get_state() == control::running_in_intersection);

        // Intersection
        ref = control_center(200, 200, 0, 0, image_processing_status_code);
        CHECK(ref.drive_mode == drive_mode::auto_critical);
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

        control_center.add_drive_instruction(control::forward, "1");
        reference_t ref = control_center(sensor_data, image_data);
        CHECK(control_center.get_state() == control::running);
        CHECK(ref.drive_mode == drive_mode::auto_nominal);
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(ref.angle == 0);
    }
}
