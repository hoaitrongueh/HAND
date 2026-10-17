#include "hand/drawing.hpp"
#include <algorithm>
#include <cmath>

namespace hand {
PathValidation validate_drawing_trajectory(const std::vector<DrawingSample>& samples,
                                           const PathLimits& limits) {
    PathValidation result;
    auto fail = [&](const char* reason) { result.reason = reason; return result; };
    if (samples.size() < 2) return fail("too_few_samples");
    if (!std::isfinite(limits.max_joint_step) || limits.max_joint_step <= 0 ||
        !std::isfinite(limits.quaternion_norm_tolerance) || limits.quaternion_norm_tolerance < 0)
        return fail("invalid_limits");
    for (int j = 0; j < 7; ++j) {
        if (!std::isfinite(limits.lower[j]) || !std::isfinite(limits.upper[j]) ||
            limits.lower[j] >= limits.upper[j] || !std::isfinite(limits.velocity[j]) ||
            limits.velocity[j] <= 0 || !std::isfinite(limits.acceleration[j]) || limits.acceleration[j] <= 0)
            return fail("invalid_limits");
    }
    if (samples.front().time != 0) return fail("start_time");
    if (samples.front().pen_down || samples.back().pen_down) return fail("pen_not_safe_at_boundary");
    ArmJoints previous_velocity{};
    double previous_dt = 0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        result.sample = i;
        const auto& s = samples[i];
        if (!std::isfinite(s.time) || (i && s.time <= samples[i - 1].time))
            return fail("nonmonotonic_time");
        if (!s.tool.position.allFinite() || !s.tool.orientation.coeffs().allFinite() ||
            std::abs(s.tool.orientation.norm() - 1) > limits.quaternion_norm_tolerance)
            return fail("invalid_pose");
        if (s.stroke < -1 || (s.pen_down && s.stroke < 0)) return fail("invalid_pen_state");
        if (i && s.pen_down && samples[i-1].pen_down && s.stroke != samples[i-1].stroke)
            return fail("stroke_change_with_pen_down");
        const double dt = i ? s.time - samples[i-1].time : 0;
        for (int j = 0; j < 7; ++j) {
            if (!std::isfinite(s.joints[j])) return fail("nonfinite_joint");
            if (s.joints[j] < limits.lower[j] || s.joints[j] > limits.upper[j])
                return fail("joint_limit");
            if (!i) continue;
            const double step = std::abs(s.joints[j] - samples[i-1].joints[j]);
            result.max_joint_step = std::max(result.max_joint_step, step);
            if (step > limits.max_joint_step) return fail("joint_discontinuity");
            const double velocity = (s.joints[j] - samples[i-1].joints[j]) / dt;
            result.max_velocity = std::max(result.max_velocity, std::abs(velocity));
            if (std::abs(velocity) > limits.velocity[j] * (1 + 1e-6)) return fail("velocity_limit");
            if (i > 1) {
                const double acceleration = 2 * std::abs(velocity - previous_velocity[j]) / (dt + previous_dt);
                result.max_acceleration = std::max(result.max_acceleration, acceleration);
                if (acceleration > limits.acceleration[j] * (1 + 1e-6)) return fail("acceleration_limit");
            }
            previous_velocity[j] = velocity;
        }
        previous_dt = dt;
    }
    result.valid = true;
    result.reason = "valid";
    return result;
}
}
