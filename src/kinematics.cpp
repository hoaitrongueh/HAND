#include "hand/kinematics.hpp"
#include <algorithm>
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
bool inverse_kinematics(double l1, double l2, double x, double y,
                        JointAngles2D &result, bool elbow_up) {
    if (l1 <= 0 || l2 <= 0) {
        return false;
    }
    double cos_theta2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    const double cosine_tolerance = 1e-12;
    if (!std::isfinite(cos_theta2) ||
        cos_theta2 < -1 - cosine_tolerance || cos_theta2 > 1 + cosine_tolerance) {
        return false;
    }

    // Clamp only small rounding errors after rejecting unreachable targets.
    cos_theta2 = std::clamp(cos_theta2, -1.0, 1.0);
    double theta2 = std::acos(cos_theta2);
    if (!elbow_up) {
        theta2 = -theta2;
    }
    double alpha = std::atan2(y, x);
    double beta = std::atan2(l2 * std::sin(theta2), l1 + l2 * std::cos(theta2));
    double theta1 = alpha - beta;

    result = {theta1, theta2};
    return true;
}
} // namespace hand
