#ifndef FILTER_H
#define FILTER_H

/* A simple low pass filter.
 *
 * Initiate with: Filter<T> f{LEN, DEFAULT_VALUE};
 *
 * Use like this: T ret_val = f(NEW_VALUE);
 * Where retval will be the average ov the LEN last values.
 */

template <class T>
class Filter {
public:
    Filter(size_t len, T const default_value): len{len} {
        memory.resize(len, default_value);
    }
    T operator()(T const value) {
        memory.at(ptr) = value;
        ptr = (ptr + 1) % len;
        T sum{0};
        for (unsigned i{0}; i<len; ++i) {
            sum += memory.at(i);
        }
        return sum/len;
    }

private:
    size_t len;
    unsigned ptr{0};
    std::vector<T> memory{};
};
#endif  // FILTER_H
