// detector.hpp
#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <deque>
#include <filesystem>
#include <algorithm>
#include <iostream>

class Detector {
public:
    Detector(const std::string& image_folder,
             const std::string& output_csv,
             float dt = 0.033f)
        : image_folder_(image_folder),
          output_csv_(output_csv),
          dt_(dt) {}

    void run();  // 执行检测和轨迹预测

private:
    std::string image_folder_;
    std::string output_csv_;
    float dt_;
    std::deque<cv::Point3f> predicted_history_;

    void sortCorners(std::vector<cv::Point2f>& corners);
    bool isValidPair(const cv::RotatedRect& l1, const cv::RotatedRect& l2, float angleThresh = 10.0, float heightRatioThresh = 0.15);
    bool isBetweenOthers(const cv::RotatedRect& l1, const cv::RotatedRect& l2, const std::vector<cv::RotatedRect>& allLights);
};

#endif // DETECTOR_HPP