#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <vector>

namespace hand {

struct ToolPose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};
using ArmJoints = std::array<double, 7>;
struct DrawingSample {
    double time = 0;
    ArmJoints joints{};
    ToolPose tool;
    bool pen_down = false;
    int stroke = -1;
};
struct PathLimits {
    ArmJoints lower, upper, velocity, acceleration;
    double max_joint_step = .15;
    double quaternion_norm_tolerance = 1e-6;
};
struct PathValidation {
    bool valid = false;
    std::size_t sample = 0;
    std::string reason;
    double max_joint_step = 0;
    double max_velocity = 0;
    double max_acceleration = 0;
};

PathValidation validate_drawing_trajectory(const std::vector<DrawingSample>& samples,
                                           const PathLimits& limits);
}
