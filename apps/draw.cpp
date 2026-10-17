#include "hand/execution.hpp"
#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 12) { std::cerr << "Usage: hand_draw MODEL.mjb TRAJECTORY.csv OUTPUT_DIR VELOCITY ACCELERATION PLANE_X PLANE_Y PLANE_Z NORMAL_X NORMAL_Y NORMAL_Z\n"; return 2; }
    try {
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_loadModel(argv[1], nullptr), mj_deleteModel);
        if (!model) throw std::runtime_error("Cannot load binary drawing model (matching MuJoCo version required)");
        const auto samples = hand::read_drawing_trajectory(argv[2]);
        hand::execute_drawing(model.get(), samples, argv[3], std::stod(argv[4]), std::stod(argv[5]),
            Eigen::Vector3d(std::stod(argv[6]), std::stod(argv[7]), std::stod(argv[8])),
            Eigen::Vector3d(std::stod(argv[9]), std::stod(argv[10]), std::stod(argv[11])));
        std::cout << "Completed actuator execution: " << samples.size()-1 << " physics steps\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
