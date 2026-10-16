#pragma once

#include <Eigen/Core>
#include <mujoco/mujoco.h>
#include <limits>
#include <vector>

namespace hand {
enum class IkTermination {
    converged,
    joint_limit,
    iteration_limit,
    missing_joint,
    invalid_model,
    invalid_input,
    invalid_options,
    invalid_numerics,
    line_search_failed
};
const char* to_string(IkTermination reason);

struct IkOptions {
    double damping = 0.05; // finite, >= 0
    double step_size = 0.5; // finite, > 0; halved over at most ten trials per update
    double tolerance = 0.001; // finite, >= 0
    int max_iterations = 100; // > 0; bounds committed updates
};

// One recorded candidate, including limit rejections (which keep both errors equal).
// Numerical failures can return before a candidate is recorded.
// A candidate is accepted if it reaches tolerance or improves error by >= 1e-10 m.
struct IkStep {
    double error_before;
    double error_after;
    bool accepted;
};

struct IkResult {
    bool success = false;
    double final_error = std::numeric_limits<double>::quiet_NaN();
    int iterations = 0; // committed updates; rejected trials only appear in steps
    IkTermination reason = IkTermination::iteration_limit;
    std::vector<IkStep> steps;
};

// Caller supplies matching Panda model/data with FK synchronized by mj_forward.
// Rejected trials restore arm qpos and recompute FK; earlier accepted updates remain.
// joint_limit: invalid starting arm range, or every attempted trial violates limits.
// line_search_failed: no accepted trial, with a feasible candidate or no trial above the alpha floor.
IkResult solve_panda_ik(const mjModel* model, mjData* data,
                        const Eigen::Vector3d& target, const char* body = "hand",
                        const IkOptions& options = {});
}
