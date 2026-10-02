#include "controller.hpp"

// PID 构造函数
PIDcontroller::PIDcontroller(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd),
      integral_limit_(20.0),
      output_limit_(20.0),
      integral_(2, 0.0),
      gyro_(2, 0.0) {};

// 计算输出量
Vector_u PIDcontroller::computeControl(const Vector_x& x, double dt) {
    std::vector<double> errors = {x[4], x[5]};

    Vector_u u;

    for (int i=0; i<2; i++) {
        integral_[i] += errors[i] * dt;
        if (integral_[i] > integral_limit_)
            integral_[i] = integral_limit_;
        if (integral_[i] < -integral_limit_)
            integral_[i] = -integral_limit_;

        gyro_[i] = x[i+6];

        u[i] =
            kp_ * errors[i] +
            ki_ * integral_[i] +
            kd_ * gyro_[i];

        if (u[i] > output_limit_) u[i] = output_limit_;
        if (u[i] < -output_limit_) u[i] = -output_limit_;
    }

    return u;
}