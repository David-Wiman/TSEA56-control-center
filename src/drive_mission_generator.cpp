#include "map_node.h"
#include "drive_mission_generator.h"
#include "control_center.h"

#include <list>
#include <vector>

using namespace std;

DriveMissionGenerator::DriveMissionGenerator(list<MapNode*> optimal_route)
: drive_mission{}, drive_mission_nodes{} {
    // Copy list content to a vector for random-access
    vector<MapNode*> optimal_route_vector(optimal_route.size());
    std::copy(optimal_route.begin(), optimal_route.end(), optimal_route_vector.begin());

    // For every element in vector, get the name of the following element
    for (unsigned int i = 0; i < optimal_route_vector.size()-1; i++) {
        string next_node_name = optimal_route_vector[i+1]->get_name();

        bool left_neighbour_exist = optimal_route_vector[i]->get_left().node != nullptr;
        bool right_neighbour_exist = optimal_route_vector[i]->get_right().node != nullptr;

        if (left_neighbour_exist && !right_neighbour_exist) {
            string left_neighbour_name = optimal_route_vector[i]->get_left().node->get_name();
            if (next_node_name != left_neighbour_name) {
                // The next node is not a neighbour
                continue;
            }
        } else if ((left_neighbour_exist) && (right_neighbour_exist)) {
            string left_neighbour_name = optimal_route_vector[i]->get_left().node->get_name();
            string right_neighbour_name = optimal_route_vector[i]->get_right().node->get_name();
            if ((next_node_name != left_neighbour_name) && (next_node_name != right_neighbour_name)) {
                // The next node is neither of the neighbours 
                continue;
            }
        }

        if ((!left_neighbour_exist) != (!right_neighbour_exist)) {
            // If exactly one of the neighbours exist, push_back forward
            drive_mission.push_back(instruction::forward);
        } else if ((left_neighbour_exist) && (right_neighbour_exist)) {
            // Both neighbours exist
            string left_neighbour_name = optimal_route_vector[i]->get_left().node->get_name();
            string right_neighbour_name = optimal_route_vector[i]->get_right().node->get_name();
            if (left_neighbour_name == next_node_name) {
                // If next node matches name, push_back direction
                drive_mission.push_back(instruction::left);
            } else {
                drive_mission.push_back(instruction::right);
            }
        }
    }
}

DriveMissionGenerator::DriveMissionGenerator(vector<MapNode*> nodes_vector, string stop_node_name)
: drive_mission{}, drive_mission_nodes{} {
    // Sort nodes_vector in a drivable order
     unsigned int i = 0;
     int k = 0;
    while (i < nodes_vector.size()-1) {
        if (nodes_vector[i]->get_left().node != nullptr) {
            // Left exists
            if (nodes_vector[i]->get_left().node->get_name() != nodes_vector[i+1]->get_name()) {
                // Left is not correct
                if (nodes_vector[i]->get_right().node != nullptr) {
                    // Right exists
                    if (nodes_vector[i]->get_right().node->get_name() != nodes_vector[i+1]->get_name()) {
                        // Right is not correct
                        nodes_vector.push_back(nodes_vector[i]);
                        nodes_vector.erase(nodes_vector.begin()+i);
                        // swap(nodes_vector[i],nodes_vector[i+1]);
                    } else {}
                } else {
                    nodes_vector.push_back(nodes_vector[i]);
                    nodes_vector.erase(nodes_vector.begin()+i);
                    // swap(nodes_vector[i],nodes_vector[i+1]);
                }
            }
        }
        if (k == 6) {
            break;
        }
        if (i == nodes_vector.size()-1) {
            k += 1;
        } else {
            i++;
        }
    }

    MapNode *active_node = nodes_vector[0];

    // For every element in vector
    for (unsigned int i = 0; i < nodes_vector.size()-1; i++) {

        // Get active node and the of the following node
        MapNode *next_node = nodes_vector[i+1];
        string next_node_name = next_node->get_name();

        // Shorter names
        bool left_neighbour_exist = active_node->get_left().node != nullptr;
        bool right_neighbour_exist = active_node->get_right().node != nullptr;

        if ((!left_neighbour_exist) != (!right_neighbour_exist)) {
            // If exactly one of the neighbours exist, push_back "forward"
            if (next_node_name == active_node->get_left().node->get_name()) {
                drive_mission.push_back(instruction::forward);
                active_node = next_node;
            }
            if (next_node_name == stop_node_name) {
                // Reached destination
                return;
            }
        } else if ((left_neighbour_exist) && (right_neighbour_exist)) {
            // Both neighbours exist
            string left_neighbour_name = active_node->get_left().node->get_name();
            string right_neighbour_name = active_node->get_right().node->get_name();
            if (left_neighbour_name == next_node_name) {
                // If next node matches name, push_back direction
                drive_mission.push_back(instruction::left);
                active_node = next_node;
                if (next_node_name == stop_node_name) {
                    return;
                }
            } else if (right_neighbour_name == next_node_name) {
                drive_mission.push_back(instruction::right);
                active_node = next_node;
                if (next_node_name == stop_node_name) {
                    return;
                }
            } else {}
        }
    }
}

vector<instruction::InstructionNumber> DriveMissionGenerator::get_drive_mission() {
    return drive_mission;
}
