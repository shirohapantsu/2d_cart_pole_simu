/**********************************************
 * controller.hpp
 * 声明Controller基类和PID、LQR两个子类
 **********************************************/

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <Eigen/Core>

// 状态空间相关定义
using Vector_x = Eigen::Matrix<double,8,1>; // 状态向量
using Vector_u = Eigen::Matrix<double,2,1>; // 输入向量
using Matrix_A = Eigen::Matrix<double,8,8>; // 状态转移矩阵
using Matrix_B = Eigen::Matrix<double,2,8>; // 控制输入矩阵
using Matrix_Q = Eigen::Matrix<double,8,8>; // 状态代价矩阵
using Matrix_R = Eigen::Matrix2d;           // 控制代价矩阵
using Matrix_P = Eigen::Matrix2d;           // Riccati方程解
using Matrix_K = Eigen::Matrix<double,2,8>; // 状态反馈增益


// 控制器基类
class Controller {
public:
    virtual ~Controller() {}

    // 计算输入量
    virtual Vector_u computeControl(
        const Vector_x& x,
        int frequency
    ) = 0;
};

// PID 控制器
class PIDcontroller : public Controller {
private:
    double kp_;
    double ki_;
    double kd_;

    double integret_;
    double total_error_;

public:
    PIDcontroller(double kp, double ki, double kd);

    virtual Vector_u computeControl(
        const Vector_x& x,
        int frequency
    ) override;

};

// LQR 控制器
class LQRcontroller : public Controller {
private:

public:
    LQRcontroller();

    virtual Vector_u computeControl(
        const Vector_x& x,
        int frequency
    ) override;
};

#endif