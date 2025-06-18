#include <iostream>
#include <Eigen/Dense>
#include "KalmanFusion.h"

KalmanFusion::KalmanFusion(double dt, double process_noise_std, double measurement_noise_std): dt_(dt), I_(Eigen::Matrix4d::Identity()) {
    // 状态转移矩阵 A
    A_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // 观测矩阵 H（只观测 x, y）
    H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    // 初始化协方差矩阵 P
    P_ = Eigen::Matrix4d::Identity();

    // 初始化过程噪声 Q
    Q_ = process_noise_std * Eigen::Matrix4d::Identity();

    // 初始化观测噪声 R（融合后噪声）
    R_ = measurement_noise_std * Eigen::Matrix2d::Identity();

    // 初始化状态
    x_ << 5, 10, 0, 0;
}

// 输入为相机和雷达观测坐标，输出融合结果
Eigen::Vector4d KalmanFusion::process(const Eigen::Vector2d& cam, const Eigen::Vector2d& radar) {
    // Step 1: 融合观测 z = α*cam + (1−α)*radar
    double alpha = 0.5;  // 权重（可调）
    Eigen::Vector2d z = alpha * cam + (1 - alpha) * radar;

    // Step 2: 预测
    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;

    // Step 3: 更新
    Eigen::Vector2d y = z - H_ * x_;                          // 创新
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;       // 创新协方差
    Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse(); // 卡尔曼增益
    x_ = x_ + K * y;
    P_ = (I_ - K * H_) * P_;

    return x_;  // 返回当前状态：[x, y, vx, vy]
}

