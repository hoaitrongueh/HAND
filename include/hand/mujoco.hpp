#pragma once

#include <mujoco/mujoco.h>
#include <memory>

namespace hand {
// Owns one model and its matching data. Data is destroyed before the model.
// Resource/setup errors throw std::runtime_error; no IK policy lives here.
class MujocoSimulation {
public:
    explicit MujocoSimulation(const char* path);
    void reset_keyframe(const char* name);

    const mjModel* model() const { return model_.get(); }
    mjData* data() const { return data_.get(); }

private:
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_;
};
} // namespace hand
