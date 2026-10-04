#include "simulation.hpp"

// Simulation 构造函数
Simulation::Simulation(const std::string& model_path) {
    // 初始化mujoco
    char errors[1024] = {};
    model_ = mj_loadXML(model_path.c_str(), NULL, errors, 1024);
    if (!model_)
        throw std::string("load model error") + errors;

    // 按名称查找，避免 XML 中关键帧顺序变化后选错初始状态。
    initial_keyframe_ = mj_name2id(model_, mjOBJ_KEY, "initial_tilt");
    if (initial_keyframe_ < 0) {
        mj_deleteModel(model_);
        model_ = nullptr;
        throw std::runtime_error("model is missing initial_tilt keyframe");
    }

    data_ = mj_makeData(model_);

    // 初始化 GLFW
    int flag = glfwInit();
    if (!flag) {
        throw "fail to initialize GLFW";
    }

    // 创建窗口
    window = glfwCreateWindow(1200, 900, "cart_pole", NULL, NULL);
    if (!window) 
        throw std::string("create window failed");

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    mjv_makeScene(model_, &scn_, 2000);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);

    // 关联当前对象，供类外回调获取 ViewerParams。
    glfwSetWindowUserPointer(window, this);

    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    mj_resetDataKeyframe(model_, data_, initial_keyframe_);
    mj_forward(model_, data_);
}

// 析构函数
Simulation::~Simulation() {
    // 释放render
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);

    // 释放窗口
    if(window) glfwDestroyWindow(window);

    // 释放glfw
    glfwTerminate();

    // 释放model和data
    if(data_) mj_deleteData(data_);
    if(model_) mj_deleteModel(model_);
}

// 为GLFW函数提供参数的方法
ViewerParams Simulation::viewerParams() {
    return {model_,        data_,        cam_,  button_left,
            button_middle, button_right, lastx, lasty};
}

void Simulation::request_reset() {
    reset_requested_ = true;
}

bool Simulation::reset_if_requested() {
    if (!reset_requested_) {
        return false;
    }

    reset_requested_ = false;
    mj_resetDataKeyframe(model_, data_, initial_keyframe_);
    mj_forward(model_, data_);
    return true;
}

// 返回data->time
double Simulation::get_time() {
    return data_->time;
}

// 读取一次状态
Vector_x Simulation::get_state() {
    mj_forward(model_ ,data_);

    Vector_x x;
    x << data_->sensordata[0] , data_->sensordata[1],
         data_->sensordata[2] , data_->sensordata[3],
         data_->sensordata[4] , data_->sensordata[5],
         data_->sensordata[6] , data_->sensordata[7];

    return x;
}

// 进行一次控制
void Simulation::input(const Vector_u& u) {
    data_->ctrl[0] = u[0];
    data_->ctrl[1] = u[1];
}

// 推进一步
void Simulation::step() {
    mj_step(model_, data_);
}

// 刷新画面
void Simulation::refresh_scene() {
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model_, data_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

// 窗口关闭标记
bool Simulation::close_window() {
    return glfwWindowShouldClose(window);
}


/****************************************************************/
/****************************************************************/


// GLFW 事件函数
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    auto* sim = static_cast<Simulation*>(glfwGetWindowUserPointer(window));

    // Backspace：由主循环统一处理，确保控制器和调度状态同步重置。
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        sim->request_reset();
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    auto* sim = static_cast<Simulation*>(glfwGetWindowUserPointer(window));
    auto params = sim->viewerParams();

    // update button state
    params.button_left =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    params.button_middle =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    params.button_right =
        (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &params.lastx, &params.lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    auto* sim = static_cast<Simulation*>(glfwGetWindowUserPointer(window));
    auto params = sim->viewerParams();

    // no buttons down: nothing to do
    if (!params.button_left && !params.button_middle && !params.button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - params.lastx;
    double dy = ypos - params.lasty;
    params.lastx = xpos;
    params.lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift =
        (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
         glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (params.button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (params.button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(
        params.model, action, dx / height, dy / height, &params.camera
    );
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    auto* sim = static_cast<Simulation*>(glfwGetWindowUserPointer(window));
    auto params = sim->viewerParams();

    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(
        params.model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &params.camera
    );
}
