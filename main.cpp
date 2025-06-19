#include <iostream>
#include <Eigen/Dense>
#include "KalmanFusion.h"
#include "KalmanFilter.h"
#include <ctime>
#include <cstdlib>
#include "WriteFile.h"

using namespace std;
using namespace Eigen;

// 打印模块开关（按需设置为 1 或 0）
#define TRUE_DATA                      0
#define CAM_DATA                       0
#define CAM_KALMANFILTER_DATA          0
#define RADAR_DATA                     0
#define RADAR_KALMANFILTER_DATA        0
#define FUSION_DATA                    0
#define FUSION_KALMANFILTER_DATA       0

//物体初始值
const double initial_X = 5;
const double initial_Y = 10;

int main() {
    srand(static_cast<unsigned int>(time(nullptr)));  // 设置随机种子
    // 初始状态估计为 (0, 0)，初始协方差为单位矩阵
    Eigen::Vector2d init_estimate(initial_X, initial_Y);
    Eigen::Matrix2d init_covariance = Eigen::Matrix2d::Identity();

    KalmanFilter kf_cam(0.01, 0.1, init_estimate, init_covariance);  // Q, R, 初始估计, 初始协方差
    KalmanFilter kf_radar(0.01, 0.1, init_estimate, init_covariance);
    KalmanFusion fusion(0.1, 1e-2, 3.0);  // dt=0.1s，过程噪声小，测量噪声中等
    KalmanFilter kf_fused(0.1, 1e-3, init_estimate, init_covariance);

    vector<Eigen::Vector2d> true_vector;
    vector<Eigen::Vector2d> cam_vector;
    vector<Eigen::Vector2d> cam_Kalmanfilter_Vector;
    vector<Eigen::Vector2d> radar_vector;
    vector<Eigen::Vector2d> radar_Kalmanfilter_Vector;
    vector<Eigen::Vector2d> fused_vector;
    vector<Eigen::Vector2d> fused_Kalmanfilter_Vector;

    for (int t = 0; t < 20; ++t) {
        // 物体真实运动轨迹
        Eigen::Vector2d true_pos(initial_X + 0.2 * t, initial_Y + 0.4 * t);  // 理想运动
        true_vector.emplace_back(true_pos);

        //相机观测数据
        Eigen::Vector2d cam_obs = true_pos + Eigen::Vector2d::Random() * 0.5;
        cam_vector.emplace_back(cam_obs);

        //相机滤波后数据
        Eigen::Vector2d cam_obs_Kalmanfilter = kf_cam.update(cam_obs);
        cam_Kalmanfilter_Vector.emplace_back(cam_obs_Kalmanfilter);

        //雷达观测数据
        Eigen::Vector2d radar_obs = true_pos + Eigen::Vector2d::Random() * 0.8;
        radar_vector.emplace_back(radar_obs);

        //雷达滤波后数据
        Eigen::Vector2d radar_obs_Kalmanfilter = kf_radar.update(radar_obs);
        radar_Kalmanfilter_Vector.emplace_back(radar_obs_Kalmanfilter);

        //融合数据
        Eigen::Vector4d fused = fusion.process(cam_obs_Kalmanfilter, radar_obs_Kalmanfilter);
        fused_vector.emplace_back(fused.head<2>());

        //融合滤波后数据
        Eigen::Vector2d fused_obs_Kalmanfilter = kf_fused.update(fused.head<2>());
        fused_Kalmanfilter_Vector.emplace_back(fused_obs_Kalmanfilter);
    }

    // 数据写入文件 CLion默认生成在 cmake-build-debug中
    writeDataToFile("./data.txt", true_vector, false);
    writeDataToFile("./data.txt", cam_vector, true);
    writeDataToFile("./data.txt", cam_Kalmanfilter_Vector, true);
    writeDataToFile("./data.txt", radar_vector, true);
    writeDataToFile("./data.txt", radar_Kalmanfilter_Vector, true);
    writeDataToFile("./data.txt", fused_vector, true);
    writeDataToFile("./data.txt", fused_Kalmanfilter_Vector, true);

    //真实数据输出
    if (TRUE_DATA) {
        for (const auto& true_data : true_vector) {
            std::cout << "(" << true_data[0] << "," << true_data[1] << "),";
        }
        std::cout << endl;
    }

    //相机数据输出
    if (CAM_DATA) {
        for (const auto& cam_data : cam_vector) {
            std::cout << "(" << cam_data[0] << "," << cam_data[1] << "),";
        }
        std::cout << endl;
    }

    //相机滤波后数据输出
    if (CAM_KALMANFILTER_DATA) {
        for (const auto& cam_data : cam_Kalmanfilter_Vector) {
            std::cout << "(" << cam_data[0] << "," << cam_data[1] << "),";
        }
        std::cout << endl;
    }

    //雷达数据输出
    if (RADAR_DATA) {
        for (const auto& radar_data : radar_vector) {
            std::cout << "(" << radar_data[0] << "," << radar_data[1] << "),";
        }
        std::cout << endl;
    }

    //雷达滤波后数据输出
    if (RADAR_KALMANFILTER_DATA) {
        for (const auto& radar_data : radar_Kalmanfilter_Vector) {
            std::cout << "(" << radar_data[0] << "," << radar_data[1] << "),";
        }
        std::cout << endl;
    }

    //融合数据输出
    if (FUSION_DATA) {
        for (const auto& fused_data : fused_vector) {
            std::cout << "(" << fused_data[0] << "," << fused_data[1] << "),";
        }
        std::cout << endl;
    }

    //融合滤波后数据输出
    if (FUSION_KALMANFILTER_DATA) {
        for (const auto& fused_data : fused_Kalmanfilter_Vector) {
            std::cout << "(" << fused_data[0] << "," << fused_data[1] << "),";
        }
    }

    return 0;
}