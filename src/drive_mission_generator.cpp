#include "map_node.h"
#include "drive_mission_generator.h"
#include "control_center.h"

#include <list>
#include <vector>

using namespace std;

DriveMissionGenerator::DriveMissionGenerator(list<MapNode*> optimal_route)
: drive_mission{} {
    // Copy list content to a vector for random-access
    vector<MapNode*> optimal_route_vector(optimal_route.size());
    std::copy(optimal_route.begin(), optimal_route.end(), optimal_route_vector.begin());

    // For every element in vector, get the name of the following element
    for (unsigned int i = 0; i < optimal_route_vector.size()-1; i++) {
        string name_to_look_for = optimal_route_vector[i+1]->get_name();

        bool left_neighbour_exist = optimal_route_vector[i]->get_left().node != nullptr;
        bool right_neighbour_exist = optimal_route_vector[i]->get_right().node != nullptr;

        if ((!left_neighbour_exist) != (!right_neighbour_exist)) {
            // If exactly one of the neighbours exist, push_back forward
            drive_mission.push_back(control::forward);
        } else if ((left_neighbour_exist) && (right_neighbour_exist)) {
            // Both neighbours exist
            if (optimal_route_vector[i]->get_left().node->get_name() == name_to_look_for) {
                // If next node matches name, push_back direction
                drive_mission.push_back(control::left);
            } else {
                drive_mission.push_back(control::right);
            }
        }
    }
}

vector<int> DriveMissionGenerator::get_drive_mission() {
    return drive_mission;
}