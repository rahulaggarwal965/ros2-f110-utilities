#pragma once

#include <cmath>

inline constexpr double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

inline constexpr double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}
