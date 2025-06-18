#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double process_noise_q, double measurement_noise_r,
                           Eigen::Vector2d initial_estimate, Eigen::Matrix2d initial_covariance)
{
    x = initial_estimate;
    P = initial_covariance;

    Q = Eigen::Matrix2d::Identity() * process_noise_q;
    R = Eigen::Matrix2d::Identity() * measurement_noise_r;

    H = Eigen::Matrix2d::Identity();  // 直接观测位置
    F = Eigen::Matrix2d::Identity();  // 状态不变
}

Eigen::Vector2d KalmanFilter::update(const Eigen::Vector2d& measurement) {
    // 预测阶段
    x = F * x;
    P = F * P * F.transpose() + Q;

    // 更新阶段
    Eigen::Vector2d y = measurement - H * x;                     // 残差
    Eigen::Matrix2d S = H * P * H.transpose() + R;               // 残差协方差
    K = P * H.transpose() * S.inverse();                         // 卡尔曼增益

    x = x + K * y;                                               // 更新状态
    P = (Eigen::Matrix2d::Identity() - K * H) * P;               // 更新协方差

    return x;
}

Eigen::Vector2d KalmanFilter::getEstimate() const {
    return x;
}
