#include "math.h"

inline int index_at_angle(float angle_in_degrees, float angle_min, float angle_increment) {
    return (int)((deg2rad(angle_in_degrees) - angle_min) / angle_increment);
}

inline int angle_at_index(int index, float angle_min, float angle_increment) {
    return angle_min + angle_increment * index;
}
