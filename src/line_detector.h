#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H

namespace stop_line_state {
    enum StopLineState {close, mid, far};
}

class LineDetector {
public:
    LineDetector(int consecutive_param, int high_count_param);

    /* Return true if at a line (which it have not been at before),
     * otherwise return false.
     *
     * Note, this method must be called exactly once per program cycle. */
    bool at_line(int line_distance);

private:
    void update_state(int line_distance);
    bool decreasing(int line_distance);
    void reset();

    enum stop_line_state::StopLineState state{stop_line_state::close};
    int consecutive_decreasing_distances{0};
    int last_distance{100};
    int far_stop_counter{0};
    int consecutive_param;
    int high_count_param;
    bool signaled{false};
};

#endif  // LINE_DETECTOR_H
