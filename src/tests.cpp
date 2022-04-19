#include "catch.hpp"
#include "map_node.h"

using namespace std;
TEST_CASE("Map Node") {
    SECTION("Basics") {
        MapNode node1 = MapNode{"1", nullptr, nullptr};
        MapNode node2 = MapNode{"2", &node1, nullptr, 34};
        MapNode node3{"3", &node1, &node2};
    }
}
