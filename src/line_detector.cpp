#include "line_detector.h"
#include "constants.h"
#include "log.h"

#include <array>
#include <string>
#include <sstream>

#define SHORT_LINE_DISTANCE 5

using namespace std;

LineDetector::LineDetector(int consecutive_param, int high_count_param)
: consecutive_param{consecutive_param}, high_count_param{high_count_param} {
}

bool LineDetector::decreasing(int line_distance) {
    if (line_distance < last_distance + SHORT_LINE_DISTANCE) {
        ++consecutive_decreasing_distances;
        far_stop_counter = 0;
    } else {
        consecutive_decreasing_distances = 0;
    }
    if (consecutive_decreasing_distances > consecutive_param) {
        return true;
    } else {
        return false;
    }
}

void LineDetector::reset() {
    signaled = false;
    far_stop_counter = 0;
}

void LineDetector::update_state(int distance) {
    if (decreasing(distance)) {

        // Far range
        if (distance >= STOP_DISTANCE_FAR) {
            ++far_stop_counter;
            if (far_stop_counter > high_count_param) {
                reset();
                state = stop_line_state::far;
            }
        }

        // Mid range
        if ((distance < STOP_DISTANCE_FAR) && (distance > STOP_DISTANCE_CLOSE)) {
            reset();
            state = stop_line_state::mid;
        }

        // Close range
        if (distance <= STOP_DISTANCE_CLOSE) {
            state = stop_line_state::close;
        }
    } else {
        // Not decreasing, but allow a couple of high values before 
        // reset. This to make sure this is not just bad data.
        ++far_stop_counter;
        if (far_stop_counter > high_count_param) {
            reset();
            state = stop_line_state::far;
            //Logger::log(DEBUG, __FILE__, "Reset", "1");
        }
    }
}

bool LineDetector::at_line(int distance) {

    update_state(distance);

    bool retval{false};

    switch (state) {
        case stop_line_state::far:
        case stop_line_state::mid:
            retval = false;
            break;
        case stop_line_state::close:
            if (signaled) {
                retval = false;
            } else {
                retval = true;
                signaled = true;
            }
            break;
    }
    // Log
    array<string, 3> state_names{"close", "mid", "far"};
    stringstream ss{};
    ss << "stop_distance=" << distance
       << ", consec=" << consecutive_decreasing_distances
       << ", far_stop_count=" << far_stop_counter
       << ", state=" << state_names[state];
    Logger::log(DEBUG, __FILE__, "at_line", ss.str());

    return retval;
}

