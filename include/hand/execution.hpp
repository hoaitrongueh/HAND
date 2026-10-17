#pragma once
#include "hand/drawing.hpp"
#include <mujoco/mujoco.h>

namespace hand {
std::vector<DrawingSample> read_drawing_trajectory(const std::string& path);

void execute_drawing(const mjModel* model, const std::vector<DrawingSample>& samples,
                     const std::string& output_directory, double velocity_limit,
                     double acceleration_limit, const Eigen::Vector3d& plane_origin,
                     const Eigen::Vector3d& plane_normal);
}
