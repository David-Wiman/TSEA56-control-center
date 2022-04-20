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
        control_center(0, 200);
        CHECK(control_center.get_state() == control::running);
    }
    SECTION("Obstacle detection") {
        ControlCenter control_center{};
        CHECK(control_center.get_state() == control::stoped_at_node);
        drive_instruction_t instr{control::left, 1};
        control_center.add_drive_instruction(instr);
        control_center(STOP_DISTANCE-10, 200);
        CHECK(control_center.get_state() == control::stoped_at_obstacle);
        control_center(STOP_DISTANCE+10, 200);
        CHECK(control_center.get_state() == control::running);

        vector<int> obstacle_distances{0, 0, 100, 95, 75, 60, 55, 30, 25, 15, 10, 2, 0};
        for (auto distance : obstacle_distances) {
            reference_t ref = control_center(distance, 200);
            if (distance == 0 || distance > STOP_DISTANCE) {
                cout << distance << endl;
                CHECK(ref.speed == DEFAULT_SPEED);
                CHECK(control_center.get_state() == control::running);
            } else {
                CHECK(ref.speed == 0);
                CHECK(control_center.get_state() == control::stoped_at_obstacle);
            }
        }
    }
    SECTION("Removed obstacle, restart") {

    }

}
