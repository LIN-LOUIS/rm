#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main() {
    Size boardSize(8, 12);              // 棋盘格内角点数
    float squareSize = 20.0f;           // 每个格子的实际大小，单位 mm
    vector<vector<Point3f>> objpoints;  // 三维点
    vector<vector<Point2f>> imgpoints;  // 图像点

    VideoCapture cap(0);                // 打开默认相机
    if (!cap.isOpened()) {
        cerr << "无法打开摄像头！" << endl;
        return -1;
    }

    Mat frame, gray;
    int imageCount = 0;
    while (imageCount < 20) { // 至少拍 20 张
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners);

        if (found) {
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
                TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(frame, boardSize, corners, found);
            imshow("Corners", frame);

            if (waitKey(100) == 's') { // 按下 's' 键保存
                vector<Point3f> objp;
                for (int i = 0; i < boardSize.height; ++i)
                    for (int j = 0; j < boardSize.width; ++j)
                        objp.emplace_back(j * squareSize, i * squareSize, 0);
                objpoints.push_back(objp);
                imgpoints.push_back(corners);
                cout << "采集图像: " << ++imageCount << endl;
            }
        }

        imshow("Video", frame);
        if (waitKey(30) == 27) break; // ESC 退出
    }

    destroyAllWindows();

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;

    Size imageSize = frame.size();
    double rms = calibrateCamera(objpoints, imgpoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "重投影误差 RMS = " << rms << endl;
    cout << "相机内参矩阵: \n" << cameraMatrix << endl;
    cout << "畸变系数: \n" << distCoeffs << endl;

    // 计算平均重投影误差
    double totalError = 0;
    int totalPoints = 0;
    for (size_t i = 0; i < objpoints.size(); ++i) {
        vector<Point2f> projectedPoints;
        projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, projectedPoints);
        double err = norm(imgpoints[i], projectedPoints, NORM_L2);
        int n = (int)objpoints[i].size();
        totalError += err * err;
        totalPoints += n;
    }
    cout << "平均重投影误差 = " << sqrt(totalError / totalPoints) << endl;

    // 可选：保存参数
    FileStorage fs("camera_calib.yaml", FileStorage::WRITE);
    fs << "CameraMatrix" << cameraMatrix;
    fs << "DistCoeffs" << distCoeffs;
    fs << "RMS" << rms;
    fs.release();

    return 0;
}
