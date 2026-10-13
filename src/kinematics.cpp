#include "hand/kinematics.hpp"
#include <cmath>

namespace hand {
Position2D forward_kinematics(double l1, double l2, double rad1, double rad2) {
    double second_link_angle = rad1 + rad2;
    double x = l1 * std::cos(rad1) + l2 * std::cos(second_link_angle);
    double y = l1 * std::sin(rad1) + l2 * std::sin(second_link_angle);
    return {x, y};
}
bool is_reachable(double l1, double l2, double x, double y) {
    double r = std::hypot(x, y);
    double min_reach = std::abs(l1 - l2);
    double max_reach = l1 + l2;
    return r >= min_reach && r <= max_reach;
}
} // namespace hand
