#include "catch.hpp"
#include "map_node.h"
#include "control_center.h"

#include <iostream>

using namespace std;

TEST_CASE("Map Node") {
    SECTION("Constructors") {
        MapNode node1{"1", nullptr, nullptr};
        MapNode node2{"2", &node1, nullptr, 34};
        MapNode node3{"3", &node1, &node2};
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
        ControlCenter control_center{};
        CHECK(control_center.get_state() == control::stoped_at_node);
        drive_instruction_t instr{control::left, 1};
        control_center.add_drive_instruction(instr);
        control_center(1000, 200);
        CHECK(control_center.get_state() == control::running);
    }
    SECTION("Obstacle detection") {
        ControlCenter control_center{};
        CHECK(control_center.get_state() == control::stoped_at_node);
        drive_instruction_t instr{control::left, 1};
        control_center.add_drive_instruction(instr);
        control_center(STOP_DISTANCE_CLOSE-10, 200);
        CHECK(control_center.get_state() == control::stoped_at_obstacle);
        control_center(STOP_DISTANCE_CLOSE+10, 200);
        CHECK(control_center.get_state() == control::running);

        vector<int> obstacle_distances{1000, 1000, 100, 75, 60, 55, 30, 25, 15, 10, 2, 1000};
        for (auto distance : obstacle_distances) {
            reference_t ref = control_center(distance, 200);
            if (distance > STOP_DISTANCE_CLOSE) {
                CHECK(ref.speed == DEFAULT_SPEED);
                CHECK(control_center.get_state() == control::running);
            } else {
                CHECK(ref.speed == 0);
                CHECK(control_center.get_state() == control::stoped_at_obstacle);
            }
        }
    }
    SECTION("Stop-line detection") {
        ControlCenter control_center{};
        CHECK(control_center.get_state() == control::stoped_at_node);
        drive_instruction_t instr{control::left, 1};
        control_center.add_drive_instruction(instr);
        control_center(STOP_DISTANCE_CLOSE-10, 200);
        CHECK(control_center.get_state() == control::stoped_at_obstacle);
        control_center(STOP_DISTANCE_CLOSE+10, 200);
        CHECK(control_center.get_state() == control::running);

        int distance{-1};
        reference_t ref = control_center(1000, distance);
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running);
        distance = 100;
        ref = control_center(1000, distance);
        CHECK(ref.speed == DEFAULT_SPEED);
        CHECK(control_center.get_state() == control::running);

        distance = 30;
        ref = control_center(1000, distance);
        CHECK(ref.speed == 0);
        CHECK(control_center.get_state() == control::stoped_at_node);
    }
    SECTION("Drive Instructions") {
        ControlCenter control_center{};
        reference_t ref{};
        drive_instruction_t instr{control::forward, 1};
        control_center.add_drive_instruction(instr);

        ref = control_center(1000, 600);
        CHECK(control_center.get_finished_instruction_id() == 0);

        ref = control_center(1000, 500);
        ref = control_center(1000, 300);
        ref = control_center(1000, 200);
        CHECK(control_center.get_finished_instruction_id() == 0);

        ref = control_center(1000, STOP_DISTANCE_CLOSE);
        CHECK(control_center.get_finished_instruction_id() == 1);
        CHECK(control_center.get_finished_instruction_id() == 0);
    }
}
