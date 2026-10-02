/**********************************************
 * simulation.hpp
 * 声明simulation类，加载mujoco模型，给出物理模型参数
 * 并且完成仿真
 **********************************************/

#pragma once

#include <mujoco/mujoco.h>
#include <string>
#include "controller.hpp"
#include <GLFW/glfw3.h>

struct ViewerParams {
    const mjModel* model;
    mjData* data;
    mjvCamera& camera;

    bool& button_left;
    bool& button_middle;
    bool& button_right;

    double& lastx;
    double& lasty;
};

class Simulation {
private:
    // 仿真资源
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    // viewer资源
    GLFWwindow* window = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;

    // 鼠标按键状态
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;

    // 上一次鼠标位置
    double lastx = 0.0;
    double lasty = 0.0;


public:
    explicit Simulation(const std::string& model_path);
    ~Simulation();

    // 为GLFW函数提供参数的方法
    ViewerParams viewerParams();

    // 返回data->time
    double get_time();

    // 读取一次状态
    Vector_x get_state();

    // 进行一次控制
    void input(const Vector_u& u);

    // 推进一步
    void step();

    // 刷新画面
    void refresh_scene();
};

// GLFW 事件函数
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);
