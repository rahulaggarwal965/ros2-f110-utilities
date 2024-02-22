#pragma once

#include "math.h"
#include <vector>

namespace scan {

inline constexpr int index_at_angle(float angle_in_degrees, float angle_min, float angle_increment) {
    return (int)((deg2rad(angle_in_degrees) - angle_min) / angle_increment);
}

inline constexpr float angle_at_index(int index, float angle_min, float angle_increment) {
    return angle_min + angle_increment * index;
}

inline std::vector<float> keep_in_range(const std::vector<float> &ranges, size_t start, size_t end) {
    std::vector<float> r(ranges);
    std::fill(r.begin(), r.begin() + start, 0.0);
    std::fill(r.begin() + end + 1, r.end(), 0.0);
    return r;
}

}
