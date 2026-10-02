#include "simulation.hpp"
#include <iostream>

#define SENSOR_FREQUNCY 200
#define FRAME_RATE 60
#define MODEL_PATH "../cart_pole.xml"

double KP = 0.0;
double KI = 0.0;
double KD = 0.0;

int main(int argc, char* argv[]) {
    // 读取参数
    if(argc >1 ) {
        std::string input = argv[1];
        if(input == "--pid") {
            PIDcontroller controller(KP, KI, KD);
        }
        else if(input == "--lqr") {
            LQRcontroller controller;
        }
        else {
            std::cout << "illegal argument" << std::endl;
            return -1;
        }
    }

    

}