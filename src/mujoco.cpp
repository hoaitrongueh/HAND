#include "hand/mujoco.hpp"
#include <stdexcept>
#include <string>

namespace hand {
MujocoSimulation::MujocoSimulation(const char* path)
    : model_(nullptr, mj_deleteModel), data_(nullptr, mj_deleteData) {
    char error[1000] = " ";
    model_.reset(mj_loadXML(path, nullptr, error, sizeof(error)));
    if (!model_) {
        throw std::runtime_error(std::string("Model load failed: ") + error);
    }
    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
        throw std::runtime_error("Data not found");
    }
}

void MujocoSimulation::reset_keyframe(const char* name) {
    int home_id=mj_name2id(model_.get(),mjOBJ_KEY,name);
    if (home_id==-1) {
        throw std::runtime_error(std::string("Keyframe not found: ") + name);
    }
    mj_resetDataKeyframe(model_.get(),data_.get(),home_id);
    mj_forward(model_.get(),data_.get());
}
} // namespace hand
