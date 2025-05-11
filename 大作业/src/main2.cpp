#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

// 对角点按 左上、右上、右下、左下 顺序排序
void sortCorners(std::vector<cv::Point2f>& corners) {
    std::sort(corners.begin(), corners.end(), [](cv::Point2f a, cv::Point2f b) {
        return a.y < b.y;
    }); // 先按 y 值排序，分为上下两组
    std::vector<cv::Point2f> top = { corners[0], corners[1] };
    std::vector<cv::Point2f> bottom = { corners[2], corners[3] };

    std::sort(top.begin(), top.end(), [](cv::Point2f a, cv::Point2f b) {
        return a.x < b.x;
    }); // 上组按 x 值排序，得到左上和右上
    std::sort(bottom.begin(), bottom.end(), [](cv::Point2f a, cv::Point2f b) {
        return a.x < b.x;
    }); // 下组按 x 值排序，得到左下和右下

    // 重排顺序：左上、右上、右下、左下
    corners[0] = top[0];
    corners[1] = top[1];
    corners[2] = bottom[1];
    corners[3] = bottom[0];
}

// 判断 l1 和 l2 灯条之间是否有其他灯条遮挡
bool isBetweenOthers(const cv::RotatedRect& l1, const cv::RotatedRect& l2, const std::vector<cv::RotatedRect>& allLights) {
    cv::Point2f p1 = l1.center;
    cv::Point2f p2 = l2.center;
    cv::Rect boundRect = cv::boundingRect(std::vector<cv::Point2f>{p1, p2}); // 创建包围框

    for (const auto& other : allLights) {
        if (other.center == p1 || other.center == p2) continue;
        if (boundRect.contains(other.center)) return true; // 被遮挡
    }
    return false;
}

// 判断两灯条是否满足角度差与高度相似性约束
bool isValidPair(const cv::RotatedRect& l1, const cv::RotatedRect& l2, float angleThresh = 10.0, float heightRatioThresh = 0.15) {
    float angleDiff = std::abs(l1.angle - l2.angle);
    if (angleDiff > angleThresh) return false;

    float height1 = std::max(l1.size.height, l1.size.width);
    float height2 = std::max(l2.size.height, l2.size.width);
    float heightDiffRatio = std::abs(height1 - height2) / std::max(height1, height2);
    return heightDiffRatio <= heightRatioThresh;
}

int main() {
    // 读取视频
    cv::VideoCapture cap("/home/lin/Desktop/大作业/videos/10.mp4");
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频！" << std::endl;
        return -1;
    }

    // 视频帧率和间隔时间
    double fps = cap.get(cv::CAP_PROP_FPS);
    double dt = 1.0 / fps;

    // 相机内参
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        2065.058, 0.0, 658.910,
        0.0, 2086.886, 531.533,
        0.0, 0.0, 1.0);
    // 畸变系数
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) <<
        -0.0518, 0.2934, 0.0015, 0.0009, 0.0);

    // 装甲板真实四个角点的三维坐标（单位 cm）
    float width = 13.5f, height = 5.5f;
    std::vector<cv::Point3f> object_points = {
        {0, 0, 0}, {width, 0, 0}, {width, height, 0}, {0, height, 0}
    };

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            cap.set(cv::CAP_PROP_POS_FRAMES, 0); // 到达末尾则回到第一帧
            continue;
        }

        // 图像转 HSV 并提取蓝色区域
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(100, 120, 100), cv::Scalar(130, 255, 255), mask);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2); // 形态学闭操作

        // 寻找轮廓并提取灯条
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::RotatedRect> ledRects;

        for (auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 0 || area > 298) continue;

            cv::RotatedRect r = cv::minAreaRect(c);
            float ratio = std::max(r.size.width, r.size.height) / std::min(r.size.width, r.size.height);
            if (ratio > 1 && ratio < 6) {
                ledRects.push_back(r); // 添加合格的灯条
            }
        }

        // 灯条配对
        std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armorPairs;
        for (size_t i = 0; i < ledRects.size(); ++i) {
            for (size_t j = i + 1; j < ledRects.size(); ++j) {
                const auto& l1 = ledRects[i], l2 = ledRects[j];

                float dist = cv::norm(l1.center - l2.center);
                if (dist < 20 || dist > 300) continue;

                if (!isValidPair(l1, l2)) continue;
                if (isBetweenOthers(l1, l2, ledRects)) continue;

                armorPairs.emplace_back(l1, l2);
            }
        }

        // 遍历所有配对，执行角点提取与姿态估计
        for (const auto& pair : armorPairs) {
            std::vector<cv::Point2f> all_corners;
            cv::Point2f pts1[4], pts2[4];
            pair.first.points(pts1);
            pair.second.points(pts2);
            for (int i = 0; i < 4; ++i) all_corners.push_back(pts1[i]);
            for (int i = 0; i < 4; ++i) all_corners.push_back(pts2[i]);

            // 将 8 个角点配对为 4 对（距离最近）
            std::vector<std::pair<cv::Point2f, cv::Point2f>> matchedPairs;
            std::vector<bool> used(8, false);
            for (int i = 0; i < 8; ++i) {
                if (used[i]) continue;
                float minDist = std::numeric_limits<float>::max();
                int minIdx = -1;
                for (int j = i + 1; j < 8; ++j) {
                    if (used[j]) continue;
                    float dist = cv::norm(all_corners[i] - all_corners[j]);
                    if (dist < minDist) {
                        minDist = dist;
                        minIdx = j;
                    }
                }
                if (minIdx != -1) {
                    used[i] = used[minIdx] = true;
                    matchedPairs.emplace_back(all_corners[i], all_corners[minIdx]);
                }
            }

            std::vector<cv::Point2f> image_points;
            for (const auto& p : matchedPairs) {
                image_points.push_back((p.first + p.second) * 0.5f); // 取每对角点中点
            }

            if (image_points.size() != 4) continue;
            sortCorners(image_points);

            // 执行 PnP 姿态估计
            cv::Mat rvec, tvec;
            bool success = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
            if (!success) continue;

            // 可视化角点与装甲框
            for (int i = 0; i < 4; ++i) {
                cv::circle(frame, image_points[i], 5, cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, std::to_string(i), image_points[i], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                cv::line(frame, image_points[i], image_points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
            }
        }

        cv::imshow("装甲板检测", frame);
        if (cv::waitKey(1) == 27) break; // 按 ESC 退出
    }
    return 0;
}
