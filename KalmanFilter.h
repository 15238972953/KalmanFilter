#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>

class KalmanFilter {
public:
    KalmanFilter(double process_noise_q, double measurement_noise_r,
                 Eigen::Vector2d initial_estimate, Eigen::Matrix2d initial_covariance);

    // 传感器提供的测量值 z = (x, y)
    Eigen::Vector2d update(const Eigen::Vector2d& measurement);

    // 获取当前估计的 (x, y)
    Eigen::Vector2d getEstimate() const;

private:
    Eigen::Vector2d x;       // 状态估计 [pos_x, pos_y]
    Eigen::Matrix2d P;       // 状态协方差矩阵
    Eigen::Matrix2d Q;       // 过程噪声协方差
    Eigen::Matrix2d R;       // 测量噪声协方差
    Eigen::Matrix2d K;       // 卡尔曼增益

    Eigen::Matrix2d H;       // 观测矩阵
    Eigen::Matrix2d F;       // 状态转移矩阵
};

#endif // KALMAN_FILTER_H
