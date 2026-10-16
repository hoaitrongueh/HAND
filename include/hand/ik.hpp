#pragma once

#include <Eigen/Core>
#include <mujoco/mujoco.h>
#include <limits>
#include <vector>

namespace hand {
enum class IkTermination { converged, joint_limit, iteration_limit, missing_joint, invalid_model };
const char* to_string(IkTermination reason);

struct IkOptions {
    double damping = 0.05;
    double step_size = 0.5;
    double tolerance = 0.001;
    int max_iterations = 100;
};

struct IkStep {
    double error_before;
    double error_after;
    bool accepted;
};

struct IkResult {
    bool success = false;
    double final_error = std::numeric_limits<double>::quiet_NaN();
    int iterations = 0; // Committed updates; rejected proposals are not updates.
    IkTermination reason = IkTermination::iteration_limit;
    std::vector<IkStep> steps;
};

// Position of the body origin in world coordinates; orientation is unconstrained.
// Caller supplies matching model/data with current mj_forward results. Only joint1..7
// qpos are committed. Retains the demo's full-nv DLS calculation, finite-input
// behavior, and pre-update convergence check for adversarial validation.
IkResult solve_panda_ik(const mjModel* model, mjData* data,
                        const Eigen::Vector3d& target, const char* body = "hand",
                        const IkOptions& options = {});
} // namespace hand
