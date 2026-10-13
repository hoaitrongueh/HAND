#pragma once
namespace hand {
struct Position2D {
    double x;
    double y;
};

Position2D forward_kinematics(double l1, double l2, double rad1, double rad2);
bool is_reachable(double l1, double l2, double x, double y);
} // namespace hand
