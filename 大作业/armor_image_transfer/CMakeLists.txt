cmake_minimum_required(VERSION 3.10)
project(ArmorDetection)

# 启用 C++17 标准，确保支持 std::filesystem
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 设置头文件搜索路径
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

# 收集源文件（可扩展）
set(SOURCES
    ./src/main.cpp
    ./include/detector.cpp
    ./include/recv_image.hpp
    ./include/send_image.hpp
    ./include/visualization.hpp
    ./include/detector.hpp
)

# 构建可执行文件
add_executable(armor_detector ${SOURCES})

# 链接 OpenCV 和必要的标准库
target_link_libraries(armor_detector ${OpenCV_LIBS})

# 对 GCC/Clang 增加 filesystem 支持
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_link_libraries(armor_detector stdc++fs)
endif()
