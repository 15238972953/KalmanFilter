//
// Created by pengdefeng on 2025/6/19.
//

#include "WriteFile.h"

// 写入数据函数
void writeDataToFile(const std::string& filename, const std::vector<Eigen::Vector2d>& data, bool append) {
    // 根据 append 选择打开模式
    std::ios_base::openmode mode = std::ios::out;
    if (append) {
        mode |= std::ios::app;     // 追加模式
    } else {
        mode |= std::ios::trunc;   // 截断模式（清空原文件）
    }

    std::ofstream outfile(filename, mode);

    if (!outfile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    for (size_t i = 0; i < data.size(); ++i) {
        const auto& point = data[i];
        outfile << "(" << point.x() << "," << point.y() << ")";
        if (i != data.size() - 1) {
            outfile << ",";
        } else{
            outfile << std::endl;
        }
    }

    outfile.close();
    std::cout << "写入完成: " << filename << std::endl;
}