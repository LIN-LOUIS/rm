#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <fstream>

// 对角点按 左上、右上、右下、左下 顺序排序
// 对装甲板8个角点中选出的4个角点进行排序：左上、右上、右下、左下
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
    corners[0] = top[0]; corners[1] = top[1];
    corners[2] = bottom[1]; corners[3] = bottom[0];
}

// 判断两个灯条之间是否夹着其他灯条（基于坐标范围判断）
bool isBetweenOthers(const cv::RotatedRect& l1, const cv::RotatedRect& l2, const std::vector<cv::RotatedRect>& allLights) {
    for (const auto& light : allLights) {
        if (light.center == l1.center || light.center == l2.center) continue;
        if ((light.center.y > std::min(l1.center.y, l2.center.y) && light.center.y < std::max(l1.center.y, l2.center.y)) &&
            (light.center.x > std::min(l1.center.x, l2.center.x) && light.center.x < std::max(l1.center.x, l2.center.x))) {
            return true;
        }
    }
    return false;
}
    

// 判断两灯条是否满足角度差与高度差的要求（几何约束）
bool isValidPair(const cv::RotatedRect& l1, const cv::RotatedRect& l2, float angleThresh = 10.0, float heightRatioThresh = 0.15) {
    float angleDiff = std::abs(l1.angle - l2.angle);
    if (angleDiff > angleThresh) return false;
    float h1 = std::max(l1.size.height, l1.size.width);
    float h2 = std::max(l2.size.height, l2.size.width);
    return std::abs(h1 - h2) / std::max(h1, h2) <= heightRatioThresh;
}

int main() {
    std::deque<cv::Point3f> predicted_history;  // 存储预测的旋转中心历史轨迹
    cv::VideoCapture cap("/home/lin/Desktop/大作业/videos/12.mp4");
cv::VideoWriter writer("/home/lin/Desktop/大作业/videos/output12.mp4", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size((int)cap.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
    if (!cap.isOpened()) return -1;
    // 获取视频帧率和计算每帧时间间隔（秒）
double fps = cap.get(cv::CAP_PROP_FPS);
float dt = 1.0 / fps;

    // 相机内参矩阵（已标定）
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    2065.058, 0, 658.910,
    0, 2086.886, 531.533,
    0, 0, 1);
    // 畸变系数（5个参数：k1, k2, p1, p2, k3）
cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) <<
    -0.0518, 0.2934, 0.0015, 0.0009, 0);

    float width = 13.5f, height = 5.5f;
    std::vector<cv::Point3f> obj_pts = {{0, 0, 0}, {width, 0, 0}, {width, height, 0}, {0, height, 0}};

    // 初始化卡尔曼滤波器：状态维度6（位置xyz + 速度xyz），观测维度3（位置xyz）
cv::KalmanFilter kf(6, 3, 0);
    // 设置状态转移矩阵：假设匀速运动模型
kf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
    1,0,0,dt,0,0,
    0,1,0,0,dt,0,
    0,0,1,0,0,dt,
    0,0,0,1,0,0,
    0,0,0,0,1,0,
    0,0,0,0,0,1);
    // 设置测量矩阵：只观测位置 (x, y, z)，不观测速度
cv::setIdentity(kf.measurementMatrix);
    // 设置过程噪声协方差矩阵，调小可减少模型对速度突变的敏感度
cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2)); // 调大过程噪声，减缓收敛过快
    // 设置测量噪声协方差矩阵，调小可提高测量信任度
cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1)); // 提高测量不确定性，放缓修正强度
    // 设置后验误差协方差初值
cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

    // 初始化测量值矩阵 (x, y, z)
cv::Mat measurement = cv::Mat::zeros(3, 1, CV_32F);
    bool first_detected = false;
    // 不再保存上一帧的中心点位置
// cv::Point3f last_center;

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }

        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(100, 120, 100), cv::Scalar(130, 255, 255), mask);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::RotatedRect> leds;
        for (auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 0 || area > 298) continue;
            cv::RotatedRect r = cv::minAreaRect(c);
            float ratio = std::max(r.size.width, r.size.height) / std::min(r.size.width, r.size.height);
            if (ratio > 1 && ratio < 6) leds.push_back(r);
        }

        std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armors;
        for (size_t i = 0; i < leds.size(); ++i) {
            for (size_t j = i + 1; j < leds.size(); ++j) {
                if (cv::norm(leds[i].center - leds[j].center) < 20 ||
                    cv::norm(leds[i].center - leds[j].center) > 300) continue;
                if (!isValidPair(leds[i], leds[j])) continue;
                if (isBetweenOthers(leds[i], leds[j], leds)) continue;
                armors.emplace_back(leds[i], leds[j]);
            }
        }

        bool detected = false;
        for (const auto& pair : armors) {
            std::vector<cv::Point2f> pts;
            cv::Point2f a[4], b[4];
            pair.first.points(a);
            pair.second.points(b);
            for (int i = 0; i < 4; ++i) pts.push_back(a[i]);
            for (int i = 0; i < 4; ++i) pts.push_back(b[i]);

            std::vector<std::pair<cv::Point2f, cv::Point2f>> matched;
            std::vector<bool> used(8, false);
            for (int i = 0; i < 8; ++i) {
                if (used[i]) continue;
                float minDist = 99999; int minIdx = -1;
                for (int j = i + 1; j < 8; ++j) {
                    if (used[j]) continue;
                    float d = cv::norm(pts[i] - pts[j]);
                    if (d < minDist) { minDist = d; minIdx = j; }
                }
                if (minIdx != -1) {
                    used[i] = used[minIdx] = true;
                    matched.emplace_back(pts[i], pts[minIdx]);
                }
            }

            std::vector<cv::Point2f> img_pts;
            for (const auto& p : matched)
                img_pts.push_back((p.first + p.second) * 0.5f);
            if (img_pts.size() != 4) continue;
            sortCorners(img_pts);

            // 使用PnP算法根据2D图像坐标和3D世界坐标，结合相机内参与畸变参数，求解装甲板的旋转向量和位移向量
cv::Mat rvec, tvec;
bool ok = cv::solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs, rvec, tvec);
            if (!ok) continue;

            if (!first_detected) {
                kf.statePost.at<float>(0) = tvec.at<double>(0);
                kf.statePost.at<float>(1) = tvec.at<double>(1);
                kf.statePost.at<float>(2) = tvec.at<double>(2);
                first_detected = true;
            }

            measurement.at<float>(0) = tvec.at<double>(0);
            measurement.at<float>(1) = tvec.at<double>(1);
            measurement.at<float>(2) = tvec.at<double>(2);
            kf.correct(measurement);
            detected = true;

            std::vector<cv::Point3f> axes = { {0,0,0}, {2,0,0}, {0,2,0}, {0,0,2} };
            std::vector<cv::Point2f> proj_axes;
            cv::projectPoints(axes, rvec, tvec, camera_matrix, dist_coeffs, proj_axes);
            cv::line(frame, proj_axes[0], proj_axes[1], cv::Scalar(0,0,255), 2);
            cv::line(frame, proj_axes[0], proj_axes[2], cv::Scalar(0,255,0), 2);
            cv::line(frame, proj_axes[0], proj_axes[3], cv::Scalar(255,0,0), 2);

            for (int i = 0; i < 4; ++i) {
                cv::circle(frame, img_pts[i], 5, cv::Scalar(255, 255, 0), -1);
                cv::line(frame, img_pts[i], img_pts[(i+1)%4], cv::Scalar(0,255,255), 2);
            }
        }

        // 预测下一时刻的状态
cv::Mat prediction = kf.predict();
        cv::Point3f predicted_center(prediction.at<float>(0), prediction.at<float>(1), prediction.at<float>(2));
predicted_history.push_back(predicted_center);
if (predicted_history.size() > 50) predicted_history.pop_front();
        // 直接使用当前预测中心点
// last_center = detected ? predicted_center : last_center;

        // 使用当前帧预测中心点为坐标原点，绘制平移后的世界坐标轴（非旋转，而是以旋转中心为基准平移）
std::vector<cv::Point3f> shifted_axes = {
    {0, 0, 0},         // 原点
    {2, 0, 0},         // X方向单位向量
    {0, 2, 0},         // Y方向单位向量
    {0, 0, 2}          // Z方向单位向量
};
// 使用旋转中心作为平移向量进行变换
std::vector<cv::Point2f> axis_img;
cv::projectPoints(shifted_axes, cv::Mat::zeros(3,1,CV_64F), cv::Mat(predicted_center), camera_matrix, dist_coeffs, axis_img);
cv::line(frame, axis_img[0], axis_img[1], cv::Scalar(0, 0, 255), 2); // X 轴 红
cv::line(frame, axis_img[0], axis_img[2], cv::Scalar(0, 255, 0), 2); // Y 轴 绿
cv::line(frame, axis_img[0], axis_img[3], cv::Scalar(255, 0, 0), 2); // Z 轴 蓝
cv::circle(frame, axis_img[0], 5, cv::Scalar(0, 255, 255), -1); // 原点

// 可视化旋转中心轨迹
for (size_t i = 1; i < predicted_history.size(); ++i) {
    std::vector<cv::Point2f> trail_img;
    cv::projectPoints(std::vector<cv::Point3f>{predicted_history[i-1], predicted_history[i]}, cv::Mat::zeros(3,1,CV_64F), cv::Mat::zeros(3,1,CV_64F), camera_matrix, dist_coeffs, trail_img);
    cv::line(frame, trail_img[0], trail_img[1], cv::Scalar(100, 255, 255), 1);
}

        cv::imshow("Armor Tracking", frame);
writer.write(frame);
        if (cv::waitKey(1) == 27) break;
    }
    // 保存旋转中心的XY轨迹到CSV文件
std::ofstream ofs("/home/lin/Desktop/大作业/videos/trajectory_xy12.csv");
ofs << "x,y" << std::endl;
for (const auto& p : predicted_history) {
    ofs << p.x << "," << p.y << std::endl;
}
ofs.close();

return 0;
}