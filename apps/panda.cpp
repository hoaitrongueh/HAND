#include <mujoco/mujoco.h>
#include <iostream>
#include <stdexcept>
#include <vector>

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

    std::vector<mjtNum> jacp(3 * model->nv, 0);
    std::vector<mjtNum> jacr(3 * model->nv, 0);
    

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
    
    const mjtNum* position=data->xpos + 3 * hand_id;

    std::cout <<"Hand position:"
              <<position[0] <<","
              <<position[1] <<","
              <<position[2] <<"\n";

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}







    
