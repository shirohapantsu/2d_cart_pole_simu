#include "simulation.hpp"
#include <iostream>
#include <memory>

#define SENSOR_FREQUNCY 200
#define FRAME_RATE 60
#define MODEL_PATH "../cart_pole.xml"

double KP = 0.0;
double KI = 0.0;
double KD = 0.0;

int main(int argc, char* argv[]) {
    // 读取参数
    std::unique_ptr<Controller> controller;

    if (argc == 1) 
        controller.reset(new PIDcontroller(KP, KI, KD));

    if (argc > 1) {
        if (argv[1] == "--PID") 
            controller.reset(new PIDcontroller(KP, KI, KD));
        else if (argv[1] == "--LQR") 
            controller.reset(new LQRcontroller());
    }

}