/**********************************************
 * simulation.hpp
 * 声明simulation类，加载mojoco模型，给出物理模型参数
 * 并且完成仿真
 **********************************************/

#pragma once

#include <mujoco/mujoco.h>
#include <string>

class Simulation {
private:
    int sensor_frequncy;
    int frame_rate;
    mjModel* model_;


public:
    Simulation(std::string model);

    // 输入控制量
};
