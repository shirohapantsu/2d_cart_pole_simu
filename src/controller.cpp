#include "controller.hpp"
#include <iostream>

// PID 构造函数
PIDcontroller::PIDcontroller(double kp, double ki, double kd, double dt)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt), integral_limit_(20.0),
      output_limit_(20.0), integral_(2, 0.0), gyro_(2, 0.0) {};

// 计算输出量
Vector_u PIDcontroller::computeControl(const Vector_x& x) {
    std::vector<double> errors = {x[4], x[5]};

    Vector_u u;

    for (int i = 0; i < 2; i++) {
        integral_[i] += errors[i] * dt_;
        if (integral_[i] > integral_limit_)
            integral_[i] = integral_limit_;
        if (integral_[i] < -integral_limit_)
            integral_[i] = -integral_limit_;

        gyro_[i] = x[i + 6];

        u[i] = kp_ * errors[i] + ki_ * integral_[i] + kd_ * gyro_[i];

        if (u[i] > output_limit_)
            u[i] = output_limit_;
        if (u[i] < -output_limit_)
            u[i] = -output_limit_;
    }

    // 打印当前测量状态和限幅后的控制量。
    std::cout << "[PID] state=[" << x.transpose()
              << "] control=[" << u.transpose() << "]\n";

    return u;
}

// PID 重置
void PIDcontroller::reset() {
    integral_ = {0.0, 0.0};
    gyro_ = {0.0, 0.0};
}

// DARE 求解器
void LQRcontroller::solveDARE_Iterative(double tolerance, int max_iter) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || max_iter <= 0) {
        throw std::invalid_argument("DARE tolerance and max_iter must be positive");
    }

    P_ = Q_;

    // 解 (R + B^T P B) K = B^T P A，避免显式求逆。
    const auto compute_gain = [this]() -> Matrix_K {
        const Matrix_R S = R_ + B_.transpose() * P_ * B_;
        if (!S.allFinite()) {
            throw std::runtime_error("DARE produced a non-finite input cost matrix");
        }

        const Eigen::LLT<Matrix_R> solver(S);
        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("DARE input cost matrix is not positive definite");
        }

        const Matrix_K gain = solver.solve(B_.transpose() * P_ * A_);
        if (!gain.allFinite()) {
            throw std::runtime_error("DARE produced a non-finite feedback gain");
        }
        return gain;
    };

    for (int i = 0; i < max_iter; ++i) {
        const Matrix_K gain = compute_gain();
        // P_next = Q + A^T P A - A^T P B (R + B^T P B)^(-1) B^T P A
        Matrix_P P_next = Q_ + A_.transpose() * P_ * A_
                         - A_.transpose() * P_ * B_ * gain;
        // 消除浮点舍入造成的非对称分量，eval() 避免转置赋值别名。
        P_next = (0.5 * (P_next + P_next.transpose())).eval();
        if (!P_next.allFinite()) {
            throw std::runtime_error("DARE iteration produced a non-finite solution");
        }

        // 相对 Frobenius 范数判据；解的范数小于 1 时使用绝对容差。
        const double error = (P_next - P_).norm();
        const double scale = std::max(1.0, P_next.norm());
        P_ = P_next;
        if (error <= tolerance * scale) {
            K_ = compute_gain();
            return;
        }
    }

    throw std::runtime_error("DARE did not converge within max_iter iterations");
}

// LQR 构造函数
LQRcontroller::LQRcontroller(sysInfo& sys, Matrix_Q& Q, Matrix_R& R)
    : sys_(sys), Q_(Q), R_(R),output_limit_(20.0) {
    // 拆包
    const double M = sys_.cart_m;
    const double m = sys_.pole_m;
    const double L = sys_.pole_l;
    const double r = sys_.pole_r;
    const double g = sys_.g;

    const double c = L / 2.0;
    const double Iperp = m * (L * L + 3.0 * r * r) / 12.0;
    const double J = Iperp + m * c * c;

    /* 计算矩阵A、B */
    // 先计算连续状态下的AB
    Matrix_A Ac = Matrix_A::Zero();
    Matrix_B Bc = Matrix_B::Zero();

    const double D = (M + m) * J - m * m * c * c;

    const double a1 = -m * m * g * c * c / D;
    const double a2 = (M + m) * m * g * c / D;
    const double b1 = J / D;
    const double b2 = -m * c / D;

    Ac(0, 2) = 1.0;
    Ac(1, 3) = 1.0;
    Ac(2, 4) = a1;
    Ac(3, 5) = a1;
    Ac(4, 6) = 1.0;
    Ac(5, 7) = 1.0;
    Ac(6, 4) = a2;
    Ac(7, 5) = a2;
    Bc(2, 0) = b1;
    Bc(3, 1) = b1;
    Bc(6, 0) = b2;
    Bc(7, 1) = b2;

    // ZOH 离散化
    using MatrixAug = Eigen::Matrix<double, 10, 10>;

    MatrixAug augmented = MatrixAug::Zero();
    augmented.block<8, 8>(0, 0) = Ac;
    augmented.block<8, 2>(0, 8) = Bc;

    const MatrixAug scaled = augmented * sys_.dt;
    const MatrixAug transition = scaled.exp();

    A_ = transition.block<8, 8>(0, 0);
    B_ = transition.block<8, 2>(0, 8);

    // 求解 DARE
    solveDARE_Iterative();

    const Matrix_R S = R_ + B_.transpose() * P_ * B_;
    const Matrix_K G = B_.transpose() * P_ * A_;

    K_ = S.ldlt().solve(G);
}

Vector_u LQRcontroller::computeControl(const Vector_x& x) {
    Vector_u u = -K_ * x;

    for(int i=0;i<2;i++) {
        if (u[i] > output_limit_)
            u[i] = output_limit_;
        if (u[i] < -output_limit_)
            u[i] = -output_limit_;
    }

    // 打印当前测量状态和限幅后的控制量。
    std::cout << "[LQR] state=[" << x.transpose()
              << "] control=[" << u.transpose() << "]\n";

    return u;
}
