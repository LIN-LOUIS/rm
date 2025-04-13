cmake_minimum_required(VERSION 3.5)
project(KalmanFilter)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(Kalman ./src/main1.cpp)
target_link_libraries(Kalman Eigen3::Eigen)
