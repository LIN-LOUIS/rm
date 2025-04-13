#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

// 对角点按 左上、右上、右下、左下 排序
void sortCorners(std::vector<cv::Point2f>& corners) {
    std::sort(corners.begin(), corners.end(), [](cv::Point2f a, cv::Point2f b) {
        return a.y < b.y;
    });
    std::vector<cv::Point2f> top = { corners[0], corners[1] };
    std::vector<cv::Point2f> bottom = { corners[2], corners[3] };

    std::sort(top.begin(), top.end(), [](cv::Point2f a, cv::Point2f b) {
        return a.x < b.x;
    });
    std::sort(bottom.begin(), bottom.end(), [](cv::Point2f a, cv::Point2f b) {
        return a.x < b.x;
    });

    corners[0] = top[0];
    corners[1] = top[1];
    corners[2] = bottom[1];
    corners[3] = bottom[0];
}

int main() {
    cv::Mat src = cv::imread("/home/lin/Desktop/图像处理/image/7.jpg");
    if (src.empty()) {
        std::cerr << "图像读取失败！" << std::endl;
        return -1;
    }

    // 相机参数
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        2065.058, 0.0, 658.910,
        0.0, 2086.886, 531.533,
        0.0, 0.0, 1.0);

    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) <<
        -0.0518, 0.2934, 0.0015, 0.0009, 0.0);

    // 提取蓝色区域
    cv::Mat hsv, mask;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(100, 120, 100), cv::Scalar(130, 255, 255), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> ledRects;                                          //存储灯条旋转矩形
    for (const auto& contour : contours) {                  //遍历所有轮廓
        if (cv::contourArea(contour) < 150.0) continue;// 舍弃面积小于150的轮廓
        cv::RotatedRect rrect = cv::minAreaRect(contour);// 计算最小外接矩形
        float ratio = std::max(rrect.size.width, rrect.size.height) / std::min(rrect.size.width, rrect.size.height);        // 计算最小外接矩形的宽高比
        if (ratio < 3 || ratio > 16) continue;      // 舍弃长宽比不在3到16之间的矩形
        ledRects.push_back(rrect);//将满足条件的灯条旋转矩形加入ledRects
    }

    // 匹配灯条对
    std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armorPairs;
    for (size_t i = 0; i < ledRects.size(); ++i) {              //ledRects 是之前提取的所有可能灯条的旋转矩形集合
        for (size_t j = i + 1; j < ledRects.size(); ++j) {                  //遍历所有灯条对
            cv::Point2f center_i = ledRects[i].center;                      //获取当前灯条的旋转矩形的中心点
            cv::Point2f center_j = ledRects[j].center;                        //获取下一个灯条的旋转矩形的中心点
            float center_dist = cv::norm(center_i - center_j);          //对每一对灯条，计算它们的中心点之间的欧氏距离
            if (center_dist > 20 && center_dist < 300) {                // 自定义距离阈值
                armorPairs.emplace_back(ledRects[i], ledRects[j]);      // 如果距离在阈值范围内，则认为它们是同一装甲板上的灯条对
            }
        }
    }

    // 设置真实世界坐标（单位 cm）
    float width = 13.5f;
    float height = 5.5f;
    std::vector<cv::Point3f> object_points = {// 真实世界坐标
        {0, 0, 0},
        {width, 0, 0},
        {width, height, 0},
        {0, height, 0}
    };

    // 遍历每对装甲板
    for (const auto& pair : armorPairs) {
        std::vector<cv::Point2f> all_corners;           // 存储所有角点
        cv::Point2f pts1[4], pts2[4];                   // 存储两个灯条的四个角点
        pair.first.points(pts1);                        // 获取第一个灯条的四个角点
        pair.second.points(pts2);                       // 获取第二个灯条的四个角点

        for (int i = 0; i < 4; ++i) all_corners.push_back(pts1[i]);         // 将第一个灯条的四个角点加入all_corners
        for (int i = 0; i < 4; ++i) all_corners.push_back(pts2[i]);         // 将第二个灯条的四个角点加入all_corners
        // 配对得到中点
        std::vector<std::pair<cv::Point2f, cv::Point2f>> matchedPairs;      // 存储匹配到的角点对
        std::vector<bool> used(8, false);                                   // 标记角点是否已被使用
        for (int i = 0; i < 8; ++i) {                                       // 遍历所有角点
            if (used[i]) continue;                                         // 如果角点已被使用，跳过        
            float minDist = std::numeric_limits<float>::max();               // 初始化最小距离为最大值
            int minIdx = -1;                                                    // 初始化最小索引为-1
            for (int j = i + 1; j < 8; ++j) {                               // 遍历剩余的角点
                if (used[j]) continue;                                      // 如果角点已被使用，跳过
                float dist = cv::norm(all_corners[i] - all_corners[j]);         // 计算角点之间的欧氏距离
                if (dist < minDist) {                                           // 如果距离小于当前最小距离，更新最小距离和最小索引
                    minDist = dist;                                             // 更新最小距离
                    minIdx = j;                                                 // 更新最小索引 
                }
            }
            if (minIdx != -1) {                                                 // 如果找到了匹配的角点
                used[i] = used[minIdx] = true;                                   // 标记这两个角点为已使用
                matchedPairs.emplace_back(all_corners[i], all_corners[minIdx]); // 将匹配到的角点对加入matchedPairs
            }
        }
        std::vector<cv::Point2f> image_points;                                      // 存储匹配到的角点
        for (const auto& p : matchedPairs) {                                        // 遍历所有匹配到的角点对
            image_points.push_back((p.first + p.second) * 0.5f);                    // 计算中点并加入image_points
        }

        if (image_points.size() != 4) continue;                                                      // 如果匹配到的角点数量不为4，跳过
        sortCorners(image_points);                                                                   // 对匹配到的角点进行排序

        // PnP 解算
        cv::Mat rvec, tvec;
        cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

        // 可视化装甲板框与角点
        for (int i = 0; i < 4; ++i) {
            cv::circle(src, image_points[i], 5, cv::Scalar(0, 255, 0), -1);
            cv::putText(src, std::to_string(i), image_points[i], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            cv::line(src, image_points[i], image_points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
        }

        std::cout << "装甲板平移向量 tvec:\n" << tvec.t() << std::endl;
        std::cout << "距离: " << cv::norm(tvec) << " cm" << std::endl;

        // 欧拉角
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        double sy = std::sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
        bool singular = sy < 1e-6;
        double x, y, z;
        if (!singular) {
            x = atan2(R.at<double>(2,1), R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        } else {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }

        x *= 180.0 / CV_PI;
        y *= 180.0 / CV_PI;
        z *= 180.0 / CV_PI;

        std::cout << "欧拉角 [Roll(X), Pitch(Y), Yaw(Z)]: " << x << "°, " << y << "°, " << z << "°" << std::endl;
        std::cout << "-----------------------------\n";
    }

    cv::imshow("Armor Detection", src);
    cv::waitKey(0);
    return 0;
}
