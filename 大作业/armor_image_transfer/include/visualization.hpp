#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>

struct Point2f {
    float x, y;
};

// 从CSV文件读取轨迹并绘图保存
inline void visualizeTrajectory(const std::string& csv_path, const std::string& output_img) {
    std::ifstream infile(csv_path);
    if (!infile.is_open()) {
        std::cerr << "Failed to open file: " << csv_path << std::endl;
        return;
    }

    std::vector<Point2f> points;
    std::string line;
    std::getline(infile, line);  // Skip header
    while (std::getline(infile, line)) {
        float x, y;
        if (sscanf(line.c_str(), "%f,%f", &x, &y) == 2) {
            points.push_back({x, y});
        }
    }
    infile.close();

    if (points.empty()) {
        std::cerr << "No data points found in: " << csv_path << std::endl;
        return;
    }

    // canvas
    const int img_size = 800;
    const int margin = 50;
    cv::Mat canvas(img_size, img_size, CV_8UC3, cv::Scalar(255, 255, 255));

    // range
    float min_x = points[0].x, max_x = points[0].x;
    float min_y = points[0].y, max_y = points[0].y;
    for (const auto& p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    float range_x = max_x - min_x;
    float range_y = max_y - min_y;
    float scale = (img_size - 2 * margin) / std::max(range_x, range_y);

    auto worldToImage = [&](float x, float y) -> cv::Point {
        int px = margin + static_cast<int>((x - min_x) * scale);
        int py = img_size - margin - static_cast<int>((y - min_y) * scale);
        return cv::Point(px, py);
    };

    // 坐标轴
    cv::line(canvas, cv::Point(margin, img_size - margin), cv::Point(img_size - margin, img_size - margin), cv::Scalar(0, 0, 0), 2);
    cv::line(canvas, cv::Point(margin, img_size - margin), cv::Point(margin, margin), cv::Scalar(0, 0, 0), 2);

    // 轨迹绘制
    for (size_t i = 1; i < points.size(); ++i) {
        cv::Point pt1 = worldToImage(points[i - 1].x, points[i - 1].y);
        cv::Point pt2 = worldToImage(points[i].x, points[i].y);
        cv::line(canvas, pt1, pt2, cv::Scalar(128, 0, 0), 2);
        cv::circle(canvas, pt1, 3, cv::Scalar(0, 0, 255), -1);
    }
    cv::circle(canvas, worldToImage(points.back().x, points.back().y), 3, cv::Scalar(0, 0, 255), -1);

    // 文本
    cv::putText(canvas, "XY Trajectory of Armor Rotation Center", cv::Point(60, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(20, 20, 20), 2);
    cv::putText(canvas, "X", cv::Point(img_size - margin - 30, img_size - margin + 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
    cv::putText(canvas, "Y", cv::Point(margin - 30, margin + 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // 保存
    cv::imwrite(output_img, canvas);
    std::cout << "Trajectory plot saved to: " << output_img << std::endl;

    // 可视化
    cv::imshow("Armor Rotation Trajectory", canvas);
    cv::waitKey(0);
}

#endif // VISUALIZATION_HPP
