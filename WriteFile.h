//
// Created by pengdefeng on 2025/6/19.
//

#ifndef WRITEFILE_H
#define WRITEFILE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

// 写入数据函数
void writeDataToFile(const std::string& filename, const std::vector<Eigen::Vector2d>& data, bool append);


#endif //WRITEFILE_H
