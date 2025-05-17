#include "detector.hpp"

// 使用 std::filesystem 简化路径操作
namespace fs = std::filesystem;

// 对角点按 左上、右上、右下、左下 顺序排序
// 用于将装甲板角点按标准顺序排列，便于后续PnP估计
void Detector::sortCorners(std::vector<cv::Point2f>& corners) {
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

// 判断是否有其他灯条夹在两个待配对灯条之间
// 防止中间有遮挡干扰的装甲板识别误判
bool Detector::isBetweenOthers(const cv::RotatedRect& l1, const cv::RotatedRect& l2, const std::vector<cv::RotatedRect>& allLights) {
    for (const auto& light : allLights) {
        if (light.center == l1.center || light.center == l2.center) continue;
        if ((light.center.y > std::min(l1.center.y, l2.center.y) && light.center.y < std::max(l1.center.y, l2.center.y)) &&
            (light.center.x > std::min(l1.center.x, l2.center.x) && light.center.x < std::max(l1.center.x, l2.center.x))) {
            return true;
        }
    }
    return false;
}

// 判断两个灯条是否满足几何匹配的角度差和高度比约束
// 通常用于判断两个灯条是否为同一装甲板左右灯条
bool Detector::isValidPair(const cv::RotatedRect& l1, const cv::RotatedRect& l2, float angleThresh, float heightRatioThresh) {
    float angleDiff = std::abs(l1.angle - l2.angle);
    if (angleDiff > angleThresh) return false;
    float h1 = std::max(l1.size.height, l1.size.width);
    float h2 = std::max(l2.size.height, l2.size.width);
    return std::abs(h1 - h2) / std::max(h1, h2) <= heightRatioThresh;
}

// 主检测流程：图像遍历、灯条识别、装甲板匹配、姿态估计、卡尔曼滤波预测
void Detector::run() {
    // 读取图像路径并排序（确保帧顺序）
    std::vector<std::string> image_paths;
    for (const auto& entry : fs::directory_iterator(image_folder_)) {
        if (entry.is_regular_file()) {
            image_paths.push_back(entry.path().string());
        }
    }
    std::sort(image_paths.begin(), image_paths.end());

    // 相机内参和畸变参数（已标定）
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 2065.058, 0, 658.910, 0, 2086.886, 531.533, 0, 0, 1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -0.0518, 0.2934, 0.0015, 0.0009, 0);

    // 装甲板世界坐标系中四个角点的3D坐标（单位：cm）
    float width = 13.5f, height = 5.5f;
    std::vector<cv::Point3f> obj_pts = {{0, 0, 0}, {width, 0, 0}, {width, height, 0}, {0, height, 0}};

    // 初始化卡尔曼滤波器（6维状态：位置xyz + 速度xyz，3维测量：位置xyz）
    cv::KalmanFilter kf(6, 3, 0);
    kf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1,0,0,dt_,0,0,
        0,1,0,0,dt_,0,
        0,0,1,0,0,dt_,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1);
    cv::setIdentity(kf.measurementMatrix);
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));  // 系统噪声较小
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1)); // 测量噪声适中
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
    cv::Mat measurement = cv::Mat::zeros(3, 1, CV_32F);
    bool first_detected = false;

    // 遍历图像帧并执行检测与追踪
    for (const auto& path : image_paths) {
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) continue;

        // 转换为HSV空间并提取蓝色灯条区域
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(100, 120, 100), cv::Scalar(130, 255, 255), mask);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

        // 轮廓分析提取灯条候选
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::RotatedRect> leds;
        for (auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 0 || area > 298) continue; // 过滤噪声
            cv::RotatedRect r = cv::minAreaRect(c);
            float ratio = std::max(r.size.width, r.size.height) / std::min(r.size.width, r.size.height);
            if (ratio > 1 && ratio < 6) leds.push_back(r);
        }

        // 成对灯条构建装甲板
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

        // 对每组灯条执行姿态估计与滤波预测
        for (const auto& pair : armors) {
            std::vector<cv::Point2f> pts;
            cv::Point2f a[4], b[4];
            pair.first.points(a);
            pair.second.points(b);
            for (int i = 0; i < 4; ++i) pts.push_back(a[i]);
            for (int i = 0; i < 4; ++i) pts.push_back(b[i]);

            // 对8点找4组最近点配对，并取中点作为角点
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

            // 求解位姿（世界坐标 → 相机坐标）
            cv::Mat rvec, tvec;
            if (!cv::solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs, rvec, tvec)) continue;

            // 初始化滤波器状态
            if (!first_detected) {
                kf.statePost.at<float>(0) = tvec.at<double>(0);
                kf.statePost.at<float>(1) = tvec.at<double>(1);
                kf.statePost.at<float>(2) = tvec.at<double>(2);
                first_detected = true;
            }

            // 滤波修正
            measurement.at<float>(0) = tvec.at<double>(0);
            measurement.at<float>(1) = tvec.at<double>(1);
            measurement.at<float>(2) = tvec.at<double>(2);
            kf.correct(measurement);

            // 状态预测，记录旋转中心历史
            cv::Mat prediction = kf.predict();
            cv::Point3f predicted_center(
                prediction.at<float>(0),
                prediction.at<float>(1),
                prediction.at<float>(2));
            predicted_history_.push_back(predicted_center);
            if (predicted_history_.size() > 50) predicted_history_.pop_front();
        }
    }

    // 将预测的旋转中心轨迹保存为CSV文件，供后续可视化使用
    std::ofstream ofs(output_csv_);
    ofs << "x,y\n";
    for (const auto& p : predicted_history_) {
        ofs << p.x << "," << p.y << "\n";
    }
    ofs.close();
}