    cmake_minimum_required(VERSION 3.26)
project(ceshi)

set(CMAKE_CXX_STANDARD 17)

# 设置Eigen库的路径，请根据实际情况修改
set(EIGEN_INCLUDE_DIR D:/C++package/eigen)
# 检查Eigen库是否存在
if(EXISTS ${EIGEN_INCLUDE_DIR}/Eigen)
    include_directories(${EIGEN_INCLUDE_DIR})
    message(STATUS "Found Eigen: ${EIGEN_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Eigen not found in ${EIGEN_INCLUDE_DIR}")
endif()

add_executable(ceshi main.cpp
        KalmanFusion.h
        KalmanFusion.cpp
        KalmanFilter.cpp
        KalmanFilter.h
)

