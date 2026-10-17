#include "hand/drawing.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    try {
        hand::PathLimits limits;
        limits.lower.fill(-2); limits.upper.fill(2);
        limits.velocity.fill(1); limits.acceleration.fill(2);
        std::vector<hand::DrawingSample> path(4);
        for (int i = 0; i < 4; ++i) { path[i].time = i; path[i].joints.fill(.1 * i); }
        const auto good = hand::validate_drawing_trajectory(path, limits);
        if (!good.valid) throw std::runtime_error(good.reason);
        auto expect = [&](const std::vector<hand::DrawingSample>& p, const char* reason) {
            const auto r = hand::validate_drawing_trajectory(p, limits);
            if (r.valid || r.reason != reason) throw std::runtime_error(std::string(reason) + ": " + r.reason);
        };
        auto p = path; p[1].time = 0; expect(p, "nonmonotonic_time");
        p = path; p[1].joints[0] = 3; expect(p, "joint_limit");
        p = path; p[1].joints[0] = .5; expect(p, "joint_discontinuity");
        p = path; p[1].joints[0] = std::numeric_limits<double>::quiet_NaN(); expect(p, "nonfinite_joint");
        p = path; p[1].tool.orientation.coeffs().setZero(); expect(p, "invalid_pose");
        p = path; p.front().pen_down = true; expect(p, "pen_not_safe_at_boundary");
        p = path; p[1].pen_down = true; expect(p, "invalid_pen_state");
        p = path; p[1].pen_down = p[2].pen_down = true;
        p[1].stroke = 0; p[2].stroke = 1; expect(p, "stroke_change_with_pen_down");
        p = path; p[1].time = .01; expect(p, "velocity_limit");
        p = path; p[1].time = .1; p[2].time = .2; p[2].joints.fill(0);
        expect(p, "acceleration_limit");
        limits.velocity[0] = -1; expect(path, "invalid_limits");
        std::cout << "PASS drawing pose/joint/timing/pen contract validation\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
