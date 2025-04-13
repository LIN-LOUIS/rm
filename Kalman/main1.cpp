#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "Kalman.cpp"  //包含 KalmanFilter 类

using namespace Eigen;
using namespace std;

struct Data {
    double time;
    double value;
};

// 读取数据
vector<Data> readDataFromFile(const string &filename) {
    vector<Data> data;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return data;
    }

    double time, value;
    while (file >> time >> value) {
        data.push_back({time, value});
    }

    return data;
}

int main() {
    vector<Data> data = readDataFromFile("/home/lin/Desktop/卡尔曼波/data/homework_data_4.txt");
    if (data.empty()) {
        cerr << "No data read from file!" << endl;
        return 1;
    }

    // 初始化卡尔曼滤波器参数
    double dt = 1;
    MatrixXd A(2, 2);
    A << 1, dt,
         0, 1;
    MatrixXd H(1, 2);
    H << 1, 0;
    MatrixXd Q = MatrixXd::Identity(2, 2) * 0.0000001; 
    MatrixXd R(1, 1);
    R << 100; 
    MatrixXd P = MatrixXd::Identity(2, 2) * 1.0;
    KalmanFilter kf(dt, A, H, Q, R, P);

    // 打开 CSV 文件保存拟合数据
    ofstream outFile("fitted_results.csv");
    if (!outFile.is_open()) {
        cerr << "Failed to create output file!" << endl;
        return 1;
    }
    outFile << "Time,Measured,Fitted\n";

    // 进行卡尔曼滤波
    for (const auto &d : data) {
        VectorXd z(1);
        z << d.value;

        kf.predict();
        kf.update(z);

        double fitted_value = kf.getState()[0];

        // 存入 CSV 文件
        outFile << d.time << "," << d.value << "," << fitted_value << "\n";
    }

    outFile.close();
    cout << "Fitted results saved to fitted_results.csv" << endl;
    return 0;
}
