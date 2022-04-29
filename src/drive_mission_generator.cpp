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
            drive_mission.push_back(control::forward);
        } else if ((left_neighbour_exist) && (right_neighbour_exist)) {
            // Both neighbours exist
            string left_neighbour_name = optimal_route_vector[i]->get_left().node->get_name();
            string right_neighbour_name = optimal_route_vector[i]->get_right().node->get_name();
            if (left_neighbour_name == next_node_name) {
                // If next node matches name, push_back direction
                drive_mission.push_back(control::left);
            } else {
                drive_mission.push_back(control::right);
            }
        }
    }
}

DriveMissionGenerator::DriveMissionGenerator(list<MapNode*> optimal_route, string stop_node_name)
: drive_mission{}, drive_mission_nodes{} {
    // Copy list content to a vector for random-access
    vector<MapNode*> optimal_route_vector(optimal_route.size());
    std::copy(optimal_route.begin(), optimal_route.end(), optimal_route_vector.begin());

    drive_mission_nodes.push_back(optimal_route.front());

    // For every element in vector, get the name of the following element
    for (unsigned int i = 0; i < optimal_route_vector.size()-1; i++) {
        
        MapNode* active_node = optimal_route_vector[i];
        string next_node_name = optimal_route_vector[i+1]->get_name();

        bool left_neighbour_exist = optimal_route_vector[i]->get_left().node != nullptr;
        bool right_neighbour_exist = optimal_route_vector[i]->get_right().node != nullptr;

        if (left_neighbour_exist && !right_neighbour_exist) {
            string left_neighbour_name = optimal_route_vector[i]->get_left().node->get_name();
            if (next_node_name != drive_mission_nodes.back()->get_left().node->get_name()) {
                // The next node is not a neighbour
                drive_mission.push_back(control::stop);
                continue;
            }
        } else if ((left_neighbour_exist) && (right_neighbour_exist)) {
            string left_neighbour_name = optimal_route_vector[i]->get_left().node->get_name();
            string right_neighbour_name = optimal_route_vector[i]->get_right().node->get_name();
            if ((drive_mission_nodes.back()->get_left().node != nullptr) && (drive_mission_nodes.back()->get_right().node == nullptr)) {
                if ((next_node_name != drive_mission_nodes.back()->get_left().node->get_name())) {
                    // The next node is neither of the neighbours
                    drive_mission.push_back(control::stop);
                    continue;
                }
            } else if ((drive_mission_nodes.back()->get_left().node != nullptr) && (drive_mission_nodes.back()->get_right().node != nullptr)) {
                if ((next_node_name != drive_mission_nodes.back()->get_left().node->get_name()) && (next_node_name != drive_mission_nodes.back()->get_right().node->get_name())) {
                    // The next node is neither of the neighbours
                    drive_mission.push_back(control::stop);
                    continue;
                }
            }
        }

        if ((!left_neighbour_exist) != (!right_neighbour_exist)) {
            // If exactly one of the neighbours exist, push_back forward
            drive_mission.push_back(control::forward);
            drive_mission_nodes.push_back(active_node);
            if (next_node_name == stop_node_name) {
                // Reached destination
                return;
            }
        } else if ((left_neighbour_exist) && (right_neighbour_exist)) {
            // Both neighbours exist
            string left_neighbour_name = optimal_route_vector[i]->get_left().node->get_name();
            string right_neighbour_name = optimal_route_vector[i]->get_right().node->get_name();
            if (left_neighbour_name == next_node_name) {
                // If next node matches name, push_back direction
                drive_mission.push_back(control::left);
                drive_mission_nodes.push_back(active_node);
                if (next_node_name == stop_node_name) {
                    return;
                }
            } else {
                drive_mission.push_back(control::right);
                drive_mission_nodes.push_back(active_node);
                if (next_node_name == stop_node_name) {
                    return;
                }
            }
        }
    }
    for (auto n : drive_mission_nodes) {
        cout << n->get_name();
        if (n->get_left().node != nullptr) {
            cout << "[" << n->get_left().node->get_name() << "]";
        }
        if (n->get_right().node != nullptr) {
            cout << "[" << n->get_right().node->get_name() << "] ";
        } else {
            cout << " ";
        }
    }
    cout << endl;
}

vector<int> DriveMissionGenerator::get_drive_mission() {
    return drive_mission;
}