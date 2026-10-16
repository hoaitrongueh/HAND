#include "hand/ik.hpp"
#include <Eigen/Dense>
#include <array>
#include <string>

namespace hand {
const char* to_string(IkTermination reason) {
    switch (reason) {
    case IkTermination::converged: return "converged";
    case IkTermination::joint_limit: return "joint_limit";
    case IkTermination::iteration_limit: return "iteration_limit";
    case IkTermination::missing_joint: return "missing_joint";
    case IkTermination::invalid_model: return "invalid_model";
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
    const int hand_id = mj_name2id(model, mjOBJ_BODY, body);
    if (hand_id == -1) {
        result.reason = IkTermination::invalid_model;
        return result;
    }
    const double L = options.damping;
    const double alpha = options.step_size;
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

        std::array<double, 7> proposed_angles;
        std::array<int, 7> pos_addresses;
        bool valid_step = true;
        for (int i = 0; i < 7;++i) {
            std::string joint_name = "joint" + std::to_string(i+1);
            int joint_id = mj_name2id(model,mjOBJ_JOINT,joint_name.c_str());
            if (joint_id == -1) {
                result.reason = IkTermination::missing_joint;
                return result;
            }
            int pos_index = model->jnt_qposadr[joint_id];
            int dof = model->jnt_dofadr[joint_id];
            double pos = data->qpos[pos_index];
            auto new_pos = pos + alpha * dq[dof];
            proposed_angles[i] = new_pos;
            pos_addresses[i] = pos_index;

            double lower = model->jnt_range[2 * joint_id];
            double upper = model->jnt_range[2 * joint_id + 1];

            bool limited = model->jnt_limited[joint_id];
            if (limited) {
                if (new_pos < lower || new_pos > upper) {
                    valid_step = false;
                }
            }
        }
        if (valid_step) {
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
            double error = (target - updated).norm();
            ++result.iterations;
            result.final_error = error;
            result.steps.push_back({error_magnitude, error, true});
        } else {
            result.steps.push_back({error_magnitude, error_magnitude, false});
            result.reason = IkTermination::joint_limit;
            break;
        }
    }
    return result;
}
}
