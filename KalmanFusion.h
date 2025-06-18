#ifndef KALMAN_FUSION_H
#define KALMAN_FUSION_H

#include <iostream>
#include <Eigen/Dense>

class KalmanFusion {
public:
    KalmanFusion(double dt, double process_noise_std, double measurement_noise_std);
    // 输入为相机和雷达观测坐标，输出融合结果
    Eigen::Vector4d process(const Eigen::Vector2d& cam, const Eigen::Vector2d& radar);
private:
    double dt_;
    Eigen::Matrix4d A_;  // 状态转移矩阵
    Eigen::Matrix<double, 2, 4> H_;  // 观测矩阵
    Eigen::Matrix4d Q_;  // 过程噪声协方差
    Eigen::Matrix2d R_;  // 观测噪声协方差
    Eigen::Matrix4d P_;  // 状态协方差
    Eigen::Matrix4d I_;  // 单位矩阵
    Eigen::Vector4d x_;  // 当前状态
};

#endif