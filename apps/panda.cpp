#include "hand/ik.hpp"
#include "hand/mujoco.hpp"
#include <iostream>
#include <stdexcept>

int main() {
    try {
        const char* path="simulation/models/franka_emika_panda/scene.xml";
        hand::MujocoSimulation simulation(path);
        const mjModel* model = simulation.model();
        mjData* data = simulation.data();
        int hand_id=mj_name2id(model,mjOBJ_BODY,"hand");
        if (hand_id==-1) {
            std::cerr << "Hand not found\n";
            return 1;
        }
        simulation.reset_keyframe("home");

        const mjtNum* initial = data->xpos + 3 * hand_id;
        Eigen::Vector3d ini_pos(initial[0],
                                initial[1],
                                initial[2]);
        Eigen::Vector3d target = ini_pos + Eigen::Vector3d(0.02,0,0);
        const auto result = hand::solve_panda_ik(model,data,target);
        for (std::size_t i = 0; i < result.steps.size(); ++i) {
            std::cout << "Iteration " << i << " | Error: "
                      << result.steps[i].error_before << " m\n";
        }
        std::cout << "Final error: " << result.final_error << " m | Updates: "
                  << result.iterations << " | " << hand::to_string(result.reason) << '\n';
        return 0;
    } catch (const std::runtime_error& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
