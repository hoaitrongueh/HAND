#include <mujoco/mujoco.h>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include <array>

constexpr double L = 0.05;
constexpr double alpha = 0.5;

int main() {
    const char* path="simulation/models/franka_emika_panda/scene.xml";
    char error[1000] = " ";
    mjModel* model = mj_loadXML(
        path,nullptr,error,sizeof(error)
    );
    if (!model) {
        std::cerr<<"model not found\n";
        return 1;
    }
    mjData* data = mj_makeData(model);
    if (!data) {
        std::cerr<<"data not found\n";
        mj_deleteModel(model);
        return 1;
    }
    int hand_id=mj_name2id(
        model,mjOBJ_BODY,"hand"
    );
    if (hand_id==-1) {
        std::cerr << "Hand not found\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }
    auto home_id=mj_name2id(model,mjOBJ_KEY,"home");
    if (home_id==-1) {
        std::cerr << "Home not found\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    mj_resetDataKeyframe(model,data,home_id);
    mj_forward(model,data);
    const mjtNum* initial = data->xpos + 3 * hand_id;
    Eigen::Vector3d ini_pos(initial[0],
                            initial[1],
                            initial[2]);
    Eigen::Vector3d target = ini_pos + Eigen::Vector3d(0.02,0,0);
    std::vector<mjtNum> jacp(3 * model->nv, 0);
    std::vector<mjtNum> jacr(3 * model->nv, 0);
    for (int i = 0; i<100; ++i) {
        const mjtNum* position=data->xpos + 3 * hand_id;

        std::cout <<"Hand position:"
                <<position[0] <<","
                <<position[1] <<","
                <<position[2] <<"\n";

        Eigen::Vector3d current(
            position[0],
            position[1],
            position[2]
        );
        
        Eigen::Vector3d error_vector = target - current;
        constexpr double tolerance = 1e-3;
        double error_magnitude = error_vector.norm();
        std::cout << "Iteration " << i
                  << " | Error: " << error_magnitude
                  << " m\n";
        if (error_magnitude <= tolerance) {
            std::cout << "Target reached!\n";
            break;
        }
        

            mj_jacBody(
                model,
                data,
                jacp.data(),
                jacr.data(),
                hand_id
            );

            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < model->nv; ++column) {
                    int index = row * model->nv + column;
                    std::cout<<jacp[index]<<" ";
                }
                std::cout<<"\n";
            }

            auto J = Eigen::Map<const Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>>(
                jacp.data(),3,model->nv
            );
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d A = J * J.transpose() + L*L * I;
            Eigen::Vector3d y = A.ldlt().solve(error_vector);
            Eigen::VectorXd dq = J.transpose() * y;

            std::cout << "nv: " << model->nv << "\n";
            std::cout << "DLS matrix:\n" << A << "\n";
            std::cout << "Hand position: " << current.transpose() << "\n";
            std::cout << "Joint update:\n" << dq.transpose() << "\n";
            std::array<double, 7> proposed_angles;
            std::array<int, 7> pos_addresses;
            bool valid_step = true;
            for (int i = 0; i < 7;++i) {
                std::string joint_name = "joint" + std::to_string(i+1);
                int joint_id = mj_name2id(model,mjOBJ_JOINT,joint_name.c_str());
                if (joint_id == -1) {
                    std::cerr<<"joint doesnt exist";
                    mj_deleteData(data);
                    mj_deleteModel(model);
                    return 1;
                }
                int pos_index = model->jnt_qposadr[joint_id];
                int dof = model->jnt_dofadr[joint_id];
                double pos = data->qpos[pos_index];
                auto new_pos = pos + alpha * dq[dof];
                proposed_angles[i] = new_pos;
                pos_addresses[i] = pos_index;

                double lower = model->jnt_range[2 * joint_id];
                double upper = model->jnt_range[2 * joint_id + 1];

                std::cout  << joint_name
                        << " | position address: " << pos_index
                        << " | DOF address: " << dof
                        << " | angle: " << pos
                        << "\n";
                bool limited = model->jnt_limited[joint_id];
                if (limited) {
                    if (new_pos < lower || new_pos > upper) {
                        std::cerr<<"violated";
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
                std::cout << "Initial error: " << error_vector.norm() << "\n";
                std::cout << "Updated error: " << error << "\n";
            } else {
                std::cerr<<"invalid step";
                break;
            }
        }
    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
