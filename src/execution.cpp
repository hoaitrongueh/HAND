#include "hand/execution.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace hand {
namespace {
const char* header = "time,q1,q2,q3,q4,q5,q6,q7,x,y,z,qw,qx,qy,qz,pen_down,stroke";
void require(bool valid, const std::string& message) {
    if (!valid) throw std::runtime_error(message);
}
double orientation_error(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    return a.angularDistance(b);
}
ToolPose site_pose(const mjData* data, int site) {
    ToolPose pose;
    pose.position = Eigen::Map<const Eigen::Vector3d>(data->site_xpos + 3*site);
    using Matrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    pose.orientation = Eigen::Quaterniond(Eigen::Map<const Matrix>(data->site_xmat + 9*site));
    return pose;
}
}

std::vector<DrawingSample> read_drawing_trajectory(const std::string& path) {
    std::ifstream file(path);
    require(bool(file), "Cannot open trajectory");
    auto line = [&]() {
        std::string s; std::getline(file, s);
        if (!s.empty() && s.back() == '\r') s.pop_back();
        return s;
    };
    require(line() == "# hand.trajectory.v1", "Invalid trajectory schema");
    require(line() == header, "Invalid trajectory columns");
    std::vector<DrawingSample> samples;
    std::string s;
    while (std::getline(file, s)) {
        require(samples.size() < 2000000, "Trajectory exceeds sample budget");
        if (!s.empty() && s.back() == '\r') s.pop_back();
        require(!s.empty() && s.back() != ',', "Empty trajectory field");
        std::istringstream stream(s);
        std::string field;
        std::vector<double> values;
        while (std::getline(stream, field, ',')) {
            std::size_t used = 0;
            const double value = std::stod(field, &used);
            require(used == field.size() && std::isfinite(value), "Invalid numeric trajectory field");
            values.push_back(value);
        }
        require(values.size() == 17, "Invalid trajectory row size");
        require(values[15] == 0 || values[15] == 1, "Invalid pen state");
        require(values[16] >= -1 && values[16] <= 1000000 && std::floor(values[16]) == values[16], "Invalid stroke ID");
        DrawingSample row;
        row.time = values[0];
        std::copy_n(values.begin()+1, 7, row.joints.begin());
        row.tool.position = Eigen::Vector3d(values[8], values[9], values[10]);
        row.tool.orientation = Eigen::Quaterniond(values[11], values[12], values[13], values[14]);
        row.pen_down = values[15] == 1;
        row.stroke = static_cast<int>(values[16]);
        samples.push_back(row);
    }
    return samples;
}

void execute_drawing(const mjModel* model, const std::vector<DrawingSample>& samples,
                     const std::string& output_directory, double velocity_limit, double acceleration_limit,
                     const Eigen::Vector3d& plane_origin, const Eigen::Vector3d& plane_normal) {
    require(model && model->nq == 9 && model->nv == 9 && model->nu == 8, "Expected Panda drawing model");
    require(plane_origin.allFinite() && plane_normal.allFinite() && std::abs(plane_normal.norm()-1) < 1e-6,
            "Invalid drawing plane");
    PathLimits limits;
    limits.velocity.fill(velocity_limit); limits.acceleration.fill(acceleration_limit);
    std::array<int, 7> qadr{}, dof{}, actuator{};
    for (int j = 0; j < 7; ++j) {
        const int joint = mj_name2id(model, mjOBJ_JOINT, ("joint"+std::to_string(j+1)).c_str());
        const int act = mj_name2id(model, mjOBJ_ACTUATOR, ("actuator"+std::to_string(j+1)).c_str());
        require(joint >= 0 && act >= 0, "Missing arm mapping");
        require(model->jnt_type[joint] == mjJNT_HINGE && model->jnt_limited[joint], "Invalid joint type/limits");
        require(model->actuator_trntype[act] == mjTRN_JOINT && model->actuator_trnid[2*act] == joint &&
                model->actuator_gaintype[act] == mjGAIN_FIXED && model->actuator_biastype[act] == mjBIAS_AFFINE &&
                model->actuator_gainprm[10*act] > 0 && model->actuator_gear[6*act] == 1 &&
                model->actuator_biasprm[10*act] == 0 &&
                model->actuator_biasprm[10*act+1] == -model->actuator_gainprm[10*act] &&
                model->actuator_biasprm[10*act+2] < 0 && model->actuator_ctrllimited[act], "Unsupported actuator servo");
        actuator[j] = act; qadr[j] = model->jnt_qposadr[joint]; dof[j] = model->jnt_dofadr[joint];
        limits.lower[j] = std::max(model->jnt_range[2*joint], model->actuator_ctrlrange[2*act]);
        limits.upper[j] = std::min(model->jnt_range[2*joint+1], model->actuator_ctrlrange[2*act+1]);
    }
    const auto validation = validate_drawing_trajectory(samples, limits);
    require(validation.valid, "Trajectory rejected at sample " + std::to_string(validation.sample) + ": " + validation.reason);
    const int home = mj_name2id(model, mjOBJ_KEY, "home"), site = mj_name2id(model, mjOBJ_SITE, "pen_tip");
    require(home >= 0 && site >= 0, "Missing drawing frames");
    const double dt = model->opt.timestep;
    using Data = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
    Data planning(mj_makeData(model), mj_deleteData), live(mj_makeData(model), mj_deleteData);
    require(planning && live, "Cannot allocate physics state");
    mj_resetDataKeyframe(model, planning.get(), home);
    mj_resetDataKeyframe(model, live.get(), home);
    mj_forward(model, live.get());
    for (int j = 0; j < 7; ++j)
        require(std::abs(live->qpos[qadr[j]] - samples.front().joints[j]) < 1e-8, "Initial command is not home");
    bool has_pen_down = false;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto& row = samples[i];
        require(std::abs(row.time - i*dt) < 1e-7, "Command timestamps must match physics timestep");
        for (int j = 0; j < 7; ++j) planning->qpos[qadr[j]] = row.joints[j];
        mj_forward(model, planning.get());
        const auto pose = site_pose(planning.get(), site);
        require((pose.position-row.tool.position).norm() <= 1e-4 &&
                orientation_error(pose.orientation, row.tool.orientation) <= 1e-3, "Command FK residual exceeds bound");
        for (int c = 0; c < planning->ncon; ++c)
            require(planning->contact[c].dist >= -1e-5, "Command path collision");
        if (row.pen_down) {
            has_pen_down = true;
            require(std::abs((row.tool.position-plane_origin).dot(plane_normal)) <= 1e-4, "Nonplanar pen-down command");
        }
    }
    require(has_pen_down, "Trajectory has no pen-down motion");
    std::filesystem::create_directories(output_directory);
    std::ofstream trace(output_directory + "/executed.csv");
    require(bool(trace), "Cannot write execution trace");
    trace << std::setprecision(17) << "time,stroke,pen_requested,pen_down,x,y,z,qw,qx,qy,qz";
    for (const auto* prefix : {"q", "desired_q", "ctrl", "force"})
        for (int j = 1; j <= 7; ++j) trace << ',' << prefix << j;
    trace << ",joint_error_max,cartesian_error,orientation_error\n";
    double max_joint_error = 0, max_cartesian_error = 0, max_orientation_error = 0;
    std::string failure;
    std::size_t steps = 0;
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const auto& row = samples[i];
        bool control_valid = true;
        for (int j = 0; j < 7; ++j) {
            const int act = actuator[j];
            const double gain = model->actuator_gainprm[10*act], damping = -model->actuator_biasprm[10*act+2];
            const double velocity = (row.joints[j]-samples[i-1].joints[j])/dt;


            const double command = row.joints[j] + (damping*velocity + live->qfrc_bias[dof[j]])/gain;
            if (!std::isfinite(command) || command < model->actuator_ctrlrange[2*act] || command > model->actuator_ctrlrange[2*act+1])
                control_valid = false;
            live->ctrl[act] = command;
        }
        if (!control_valid) { failure = "servo_control_range"; break; }
        mj_step(model, live.get());
        mj_forward(model, live.get());
        ++steps;
        const auto pose = site_pose(live.get(), site);
        double joint_error = 0;
        for (int j = 0; j < 7; ++j) {
            joint_error = std::max(joint_error, std::abs(live->qpos[qadr[j]]-row.joints[j]));
            if (!std::isfinite(live->qpos[qadr[j]]) || live->qpos[qadr[j]] < limits.lower[j]-1e-4 ||
                live->qpos[qadr[j]] > limits.upper[j]+1e-4) failure = "live_joint_limit";
        }
        const double cart_error = (pose.position-row.tool.position).norm();
        const double ori_error = orientation_error(pose.orientation, row.tool.orientation);
        if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() || !std::isfinite(joint_error) ||
            !std::isfinite(cart_error) || !std::isfinite(ori_error)) failure = "nonfinite_physics";
        if (joint_error > .05 || cart_error > .01 || ori_error > .05) failure = "tracking_guard";
        for (int c = 0; c < live->ncon; ++c)
            if (live->contact[c].dist < -1e-4) failure = "live_collision";
        for (int warning = 0; warning < mjNWARNING; ++warning)
            if (live->warning[warning].number) failure = "physics_warning";
        const bool marking = failure.empty() && row.pen_down && cart_error <= .002 && ori_error <= .01 &&
                             std::abs((pose.position-plane_origin).dot(plane_normal)) <= .002;
        max_joint_error = std::max(max_joint_error, joint_error);
        max_cartesian_error = std::max(max_cartesian_error, cart_error);
        max_orientation_error = std::max(max_orientation_error, ori_error);
        trace << live->time << ',' << row.stroke << ',' << row.pen_down << ',' << marking;
        for (int k = 0; k < 3; ++k) trace << ',' << pose.position[k];
        trace << ',' << pose.orientation.w() << ',' << pose.orientation.x() << ',' << pose.orientation.y() << ',' << pose.orientation.z();
        for (int j = 0; j < 7; ++j) trace << ',' << live->qpos[qadr[j]];
        for (double q : row.joints) trace << ',' << q;
        for (int j = 0; j < 7; ++j) trace << ',' << live->ctrl[actuator[j]];
        for (int j = 0; j < 7; ++j) trace << ',' << live->actuator_force[actuator[j]];
        trace << ',' << joint_error << ',' << cart_error << ',' << ori_error << '\n';
        if (!failure.empty() || !trace) { if (failure.empty()) failure = "trace_write"; break; }
    }

    trace.close();
    if (!trace && failure.empty()) failure = "trace_write";
    std::ofstream report(output_directory + "/execution.json");
    report << std::setprecision(17) << "{\n  \"status\": \"" << (failure.empty() ? "completed" : "halted")
           << "\",\n  \"failure\": \"" << failure << "\",\n  \"steps\": " << steps
           << ",\n  \"sim_time_s\": " << live->time << ",\n  \"joint_tracking_max_rad\": " << max_joint_error
           << ",\n  \"cartesian_tracking_max_m\": " << max_cartesian_error
           << ",\n  \"orientation_tracking_max_rad\": " << max_orientation_error << "\n}\n";
    report.close();
    require(bool(report), "Cannot save execution report");
    require(failure.empty(), "Execution halted: " + failure);
}
}
