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

    corners[0] = top[0];     // 左上
    corners[1] = top[1];     // 右上
    corners[2] = bottom[1];  // 右下
    corners[3] = bottom[0];  // 左下
}

int main() {
    cv::Mat src = cv::imread("/home/lin/Desktop/图像处理/image/21.jpg");
    if (src.empty()) {
        std::cerr << "图像读取失败！" << std::endl;
        return -1;
    }

    // 相机内参与畸变参数
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
        2065.0580175762857, 0.0, 658.9098266395495,
        0.0, 2086.886458338243, 531.5333174739342,
        0.0, 0.0, 1.0);

    cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << 
        -0.051836613762195866, 0.29341513924119095,
        0.001501183796729562, 0.0009386915104617738, 0.0);

    // 提取蓝色区域
    cv::Mat hsv, mask;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(100, 120, 100), cv::Scalar(130, 255, 255), mask);

    // 闭运算降噪
    cv::Mat morph;
    cv::morphologyEx(mask, morph, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

    // 提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morph, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> ledRects;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 200.0) continue;

        cv::RotatedRect rrect = cv::minAreaRect(contour);
        float width = rrect.size.width;
        float height = rrect.size.height;
        float ratio = std::max(width, height) / std::min(width, height);
        if (ratio < 2.5) continue;

        ledRects.push_back(rrect);
    }
    // 仅当检测到两个灯条时，组合成装甲板角点
    if (ledRects.size() == 2) {
        std::vector<cv::Point2f> all_corners;
        for (auto& rect : ledRects) {
            cv::Point2f pts[4];
            rect.points(pts);
            for (int i = 0; i < 4; ++i) {
                all_corners.push_back(pts[i]);
            }
        }
        // 对8个点进行配对，选出4对距离最近的点对，取中点作为角点
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
        
        // 取每对中点作为角点
        std::vector<cv::Point2f> image_points;
        for (const auto& pair : matchedPairs) {
            cv::Point2f mid = (pair.first + pair.second) * 0.5f;
            image_points.push_back(mid);
        }
        

        sortCorners(image_points);

        for (int i = 0; i < 4; ++i) {
            cv::circle(src, image_points[i], 5, cv::Scalar(0, 255, 0), -1);
            cv::putText(src, std::to_string(i), image_points[i], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }

        std::cout << "图像角点坐标：" << std::endl;
        for (const auto& pt : image_points) {
            std::cout << pt << std::endl;
        }

        // 设置装甲板真实世界坐标（单位 cm）
        float width = 12.5f;
        float height = 5.5f;
        std::vector<cv::Point3f> object_points = {
            cv::Point3f(0, 0, 0),
            cv::Point3f(width, 0, 0),
            cv::Point3f(width, height, 0),
            cv::Point3f(0, height, 0)
        };
        // 可视化角点
    for (int i = 0; i < 4; ++i) {
        cv::circle(src, image_points[i], 5, cv::Scalar(0, 255, 0), -1);
        cv::putText(src, std::to_string(i), image_points[i], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

        // 连线：将四个点首尾相连形成一个装甲板矩形框
        cv::line(src, image_points[i], image_points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);  // 蓝色边框
    }

        // 解算PnP
        cv::Mat rvec, tvec;
        cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

        std::cout << "\n平移向量 tvec:\n" << tvec << std::endl;

        // 计算距离
        double distance = cv::norm(tvec);
        std::cout << "装甲板到相机的距离: " << distance << " cm" << std::endl;

        // 欧拉角输出
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
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

        x = x * 180.0 / CV_PI;
        y = y * 180.0 / CV_PI;
        z = z * 180.0 / CV_PI;

        std::cout << "欧拉角（角度制）:\n";
        std::cout << "Pitch (X): " << x << "°\n";
        std::cout << "Yaw   (Y): " << y << "°\n";
        std::cout << "Roll  (Z): " << z << "°\n";

    } else {
        std::cout << "未找到两个蓝灯条，找到数量：" << ledRects.size() << std::endl;
    }

    cv::imshow("Detected Armor Corners", src);
    cv::waitKey(0);
    return 0;
}
