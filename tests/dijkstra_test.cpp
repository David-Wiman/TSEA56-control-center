#include "dijkstra_solver.h"
#include "map_graph.h"
#include "map_node.h"

#include <string>
#include <json.hpp>


using namespace std;
using json = nlohmann::json;

int main() {
    solver = DijkstraSolver();
    string map_string = "{\"MapGraph\": {\"A1\": 2}, {\"B2\": 3}}";

    json json_map{};
    try {
        json_map = json::parse(map_string);
    } catch (std::invalid_argument&) {
        return -1;
    }

    map_graph = MapGraph(json_map);

    list<MapNode*> optimal_route = solver.solve("A1");

    return 0;
}