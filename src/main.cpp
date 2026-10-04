#include "simulation.hpp"
#include <memory>

#define FRAME_RATE 60.0
#define MODEL_PATH "cart_pole.xml"

double KP = 40.0;
double KI = 10.0;
double KD = 6.0;

Matrix_Q Q = Matrix_Q::Zero();
Matrix_R R = Matrix_R::Zero();


sysInfo cart_pole{
    1.0, 0.1, 1.0, 0.02, 9.81, 0.005
};

int main(int argc, char* argv[]) {
    // 初始化LQR控制矩阵
    Q.diagonal() << 25.0, 25.0, 4.0, 4.0,
                1459.0, 1459.0, 1.0, 1.0;
    R.diagonal() << 0.0025, 0.0025;

    // 读取参数
    std::unique_ptr<Controller> controller;

    if (argc == 1) 
        controller.reset(new PIDcontroller(KP, KI, KD, cart_pole.dt));

    if (argc > 1) {
        if (std::string(argv[1]) == "--PID") 
            controller.reset(new PIDcontroller(KP, KI, KD, cart_pole.dt));
        else if (std::string(argv[1]) == "--LQR") 
            controller.reset(new LQRcontroller(cart_pole, Q, R));
        else throw "invalid parameter";
    }

    // 初始化仿真
    Simulation simulation(MODEL_PATH);
    double scene_time = simulation.get_time();
    int ctrl_step_count = 0;
    Vector_x x = simulation.get_state();
    Vector_u u;

    // 主循环
    while (!simulation.close_window()) {
        // 同步重置物理状态、控制器历史和仿真调度。
        if (simulation.reset_if_requested()) {
            controller->reset();
            ctrl_step_count = 0;
            scene_time = simulation.get_time();
        }

        // 推进仿真
        simulation.step();
        ctrl_step_count++;

        // 进行控制
        if (ctrl_step_count >= cart_pole.dt / 0.001) {
            x = simulation.get_state();
            u = controller->computeControl(x);
            simulation.input(u);
            ctrl_step_count = 0;
        }

        // 刷新画面
        if (simulation.get_time() - scene_time >= 1.0 / FRAME_RATE) {
            simulation.refresh_scene();
            scene_time = simulation.get_time();
        }
    }

    return 0;
}
