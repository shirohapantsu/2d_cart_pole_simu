/**********************************************
 * controller.hpp
 * 声明Controller基类和PID、LQR两个子类
 **********************************************/

#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <cmath>
#include <Eigen/Cholesky>
#include <algorithm>
#include <stdexcept>
#include <iostream>

// 状态空间相关定义
using Vector_x = Eigen::Matrix<double, 8, 1>; // 状态向量
using Vector_u = Eigen::Matrix<double, 2, 1>; // 输入向量
using Matrix_A = Eigen::Matrix<double, 8, 8>; // 状态转移矩阵
using Matrix_B = Eigen::Matrix<double, 8, 2>; // 控制输入矩阵
using Matrix_Q = Eigen::Matrix<double, 8, 8>; // 状态代价矩阵
using Matrix_R = Eigen::Matrix2d;             // 控制代价矩阵
using Matrix_P = Eigen::Matrix<double, 8, 8>; // Riccati方程解
using Matrix_K = Eigen::Matrix<double, 2, 8>; // 状态反馈增益

struct sysInfo {
    double cart_m;
    double pole_m;
    double pole_l;
    double pole_r;
    double g;
    double dt;
};

// 控制器基类
class Controller {
public:
    virtual ~Controller() {}

    // 计算输入量
    virtual Vector_u computeControl(const Vector_x& x) = 0;

    // 清除控制器历史状态；LQR 等无历史状态的控制器无需额外处理。
    virtual void reset() {}
};

// PID 控制器
class PIDcontroller : public Controller {
private:
    double kp_;
    double ki_;
    double kd_;
    double dt_;

    double integral_limit_;
    double output_limit_;

    std::vector<double> integral_;
    std::vector<double> gyro_;


public:
    PIDcontroller(double kp, double ki, double kd, double dt);

    virtual Vector_u computeControl(const Vector_x& x) override;

    void reset() override;
};

// LQR 控制器
class LQRcontroller : public Controller {
private:
    sysInfo sys_;
    Matrix_Q Q_;
    Matrix_R R_;
    Matrix_A A_;
    Matrix_B B_;
    Matrix_P P_;
    Matrix_K K_;

    double output_limit_;

    void solveDARE_Iterative(
        double tolerance = 1e-7, int max_iter = 10000
    ); 

public:
    LQRcontroller(sysInfo& sys, Matrix_Q& Q, Matrix_R& R);

    virtual Vector_u computeControl(const Vector_x& x) override;
};
