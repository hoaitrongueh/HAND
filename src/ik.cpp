#include "hand/ik.hpp"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <string>

namespace hand {
const char* to_string(IkTermination reason) {
    switch (reason) {
    case IkTermination::converged: return "converged";
    case IkTermination::joint_limit: return "joint_limit";
    case IkTermination::iteration_limit: return "iteration_limit";
    case IkTermination::missing_joint: return "missing_joint";
    case IkTermination::invalid_model: return "invalid_model";
    case IkTermination::invalid_input: return "invalid_input";
    case IkTermination::invalid_options: return "invalid_options";
    case IkTermination::invalid_numerics: return "invalid_numerics";
    case IkTermination::line_search_failed: return "line_search_failed";
    }
    return "unknown";
}

IkResult solve_panda_ik(const mjModel* model, mjData* data,
                        const Eigen::Vector3d& target, const char* body,
                        const IkOptions& options) {
    IkResult result;
    if (!model || !data || !body) {
        result.reason = IkTermination::invalid_model;
        return result;
    }
    if (!target.allFinite()) {
        result.reason = IkTermination::invalid_input;
        return result;
    }
    if (!std::isfinite(options.damping) || options.damping < 0.0 ||
        !std::isfinite(options.step_size) || options.step_size <= 0.0 ||
        !std::isfinite(options.tolerance) || options.tolerance < 0.0 ||
        options.max_iterations <= 0) {
        result.reason = IkTermination::invalid_options;
        return result;
    }
    const int hand_id = mj_name2id(model, mjOBJ_BODY, body);
    if (hand_id == -1) {
        result.reason = IkTermination::invalid_model;
        return result;
    }
    const double L = options.damping;
    const double step_size = options.step_size;
    std::vector<mjtNum> jacp(3 * model->nv, 0);
    std::vector<mjtNum> jacr(3 * model->nv, 0);
    for (int i = 0; i<options.max_iterations; ++i) {
        const mjtNum* position=data->xpos + 3 * hand_id;

        Eigen::Vector3d current(
            position[0],
            position[1],
            position[2]
        );

        Eigen::Vector3d error_vector = target - current;
        const double tolerance = options.tolerance;
        double error_magnitude = error_vector.norm();
        result.final_error = error_magnitude;
        if (!error_vector.allFinite() || !std::isfinite(error_magnitude)) {
            result.reason = IkTermination::invalid_numerics;
            return result;
        }

        // Validate the arm even when no update is needed to reach the target.
        std::array<double, 7> original_angles;
        std::array<double, 7> proposed_angles;
        std::array<int, 7> pos_addresses;
        std::array<int, 7> dof_addresses;
        std::array<double, 7> lower_limits;
        std::array<double, 7> upper_limits;
        std::array<bool, 7> limited_joints;
        for (int i = 0; i < 7; ++i) {
            std::string joint_name = "joint" + std::to_string(i + 1);
            int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
            if (joint_id == -1) {
                result.reason = IkTermination::missing_joint;
                return result;
            }

            pos_addresses[i] = model->jnt_qposadr[joint_id];
            dof_addresses[i] = model->jnt_dofadr[joint_id];
            original_angles[i] = data->qpos[pos_addresses[i]];
            if (!std::isfinite(original_angles[i])) {
                result.reason = IkTermination::invalid_numerics;
                return result;
            }
            lower_limits[i] = model->jnt_range[2 * joint_id];
            upper_limits[i] = model->jnt_range[2 * joint_id + 1];
            limited_joints[i] = model->jnt_limited[joint_id];
            if (limited_joints[i] &&
                (original_angles[i] < lower_limits[i] || original_angles[i] > upper_limits[i])) {
                result.reason = IkTermination::joint_limit;
                return result;
            }
        }
        if (error_magnitude <= tolerance) {
            result.success = true;
            result.reason = IkTermination::converged;
            break;
        }

        mj_jacBody(
            model,
            data,
            jacp.data(),
            jacr.data(),
            hand_id
        );

        auto J = Eigen::Map<const Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>>(
            jacp.data(),3,model->nv
        );
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d A = J * J.transpose() + L*L * I;
        Eigen::Vector3d y = A.ldlt().solve(error_vector);
        Eigen::VectorXd dq = J.transpose() * y;
        if (!dq.allFinite()) {
            result.reason = IkTermination::invalid_numerics;
            return result;
        }

        constexpr int max_retries = 10;
        constexpr double min_alpha = 1e-6;
        constexpr double min_improvement = 1e-10;

        double alpha = step_size;
        bool accepted = false;
        bool attempted_candidate = false;
        bool found_valid_candidate = false;
        for (int retry = 0; retry < max_retries; ++retry) {
            if (alpha < min_alpha) {
                break;
            }

            attempted_candidate = true;
            bool valid_step = true;
            for (int i = 0; i < 7; ++i) {
                proposed_angles[i] =
                    original_angles[i] + alpha * dq[dof_addresses[i]];
                if (!std::isfinite(proposed_angles[i])) {
                    result.reason = IkTermination::invalid_numerics;
                    return result;
                }
                if (limited_joints[i] &&
                    (proposed_angles[i] < lower_limits[i] ||
                     proposed_angles[i] > upper_limits[i])) {
                    valid_step = false;
                }
            }

            if (valid_step) {
                found_valid_candidate = true;
                for (int i = 0; i < 7; ++i) {
                    data->qpos[pos_addresses[i]] = proposed_angles[i];
                }

                mj_forward(model, data);
                const mjtNum* updated_pos = data->xpos + 3 * hand_id;
                Eigen::Vector3d updated(
                    updated_pos[0],
                    updated_pos[1],
                    updated_pos[2]
                );
                const double error = (target - updated).norm();
                if (!updated.allFinite() || !std::isfinite(error)) {
                    for (int i = 0; i < 7; ++i) {
                        data->qpos[pos_addresses[i]] = original_angles[i];
                    }
                    mj_forward(model, data);
                    result.reason = IkTermination::invalid_numerics;
                    return result;
                }
                if (error <= tolerance || error_magnitude - error >= min_improvement) {
                    accepted = true;
                    ++result.iterations;
                    result.final_error = error;
                    result.steps.push_back({error_magnitude, error, true});
                    break;
                }

                for (int i = 0; i < 7; ++i) {
                    data->qpos[pos_addresses[i]] = original_angles[i];
                }
                mj_forward(model, data);
                result.steps.push_back({error_magnitude, error, false});
            } else {
                result.steps.push_back(
                    {error_magnitude, error_magnitude, false});
            }

            alpha *= 0.5;
        }

        if (!accepted) {
            if (attempted_candidate && !found_valid_candidate) {
                result.reason = IkTermination::joint_limit;
            } else {
                result.reason = IkTermination::line_search_failed;
            }
            break;
        }

        if (result.final_error <= tolerance) {
            result.success = true;
            result.reason = IkTermination::converged;
            break;
        }
    }
    return result;
}
}
