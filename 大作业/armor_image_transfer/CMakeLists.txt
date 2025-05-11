cmake_minimum_required(VERSION 3.10)
project(ArmorImageTransfer)

set(CMAKE_CXX_STANDARD 17)

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(include)

# 构建 sender 可执行程序
add_executable(sender ./sender/send_image.cpp)
target_link_libraries(sender ${OpenCV_LIBS})

# 构建 receiver 可执行程序
add_executable(receiver ./receiver/recv_image.cpp)
target_link_libraries(receiver ${OpenCV_LIBS})
