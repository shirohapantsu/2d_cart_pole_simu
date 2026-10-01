#include "controller.hpp"

// PID 构造函数
PIDcontroller::PIDcontroller(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integret_(0.0),total_error_(0.0) {};

