#include <fstream>
#include "Kalman.cpp"  // 包含 KalmanFilter 类

int main() {
    // 假设已经读取 true_states.csv 和 measurements.csv 为 vector<VectorXd> measurements

    ifstream infile("/home/lin/Desktop/卡尔曼波/data/measurements.csv");
    ofstream outfile("/home/lin/Desktop/卡尔曼波/results/kalman_output.csv");
    outfile << "x,y,z\n";

    double dt = 1.0;  // 可根据数据时间步调整
    int n = 6; // 状态变量：x,y,z,vx,vy,vz
    int m = 3; // 观测变量：x,y,z

    MatrixXd A = MatrixXd::Identity(n, n);
    for (int i = 0; i < 3; ++i)
        A(i, i + 3) = dt;

    MatrixXd H(m, n);
    H.setZero();
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;

    MatrixXd Q = MatrixXd::Identity(n, n) * 0.1;
    MatrixXd R = MatrixXd::Identity(m, m) * 1.0;
    MatrixXd P = MatrixXd::Identity(n, n) * 1;

    KalmanFilter kf(dt, A, H, Q, R, P);

    string line;
    getline(infile, line); // skip header
    while (getline(infile, line)) {
        stringstream ss(line);
        string val;
        VectorXd z(m);
        for (int i = 0; i < m; ++i) {
            getline(ss, val, ',');
            z(i) = stod(val);
        }
        kf.predict();
        kf.update(z);

        VectorXd state = kf.getState();
        outfile << state(0) << "," << state(1) << "," << state(2) << "\n";
    }

    infile.close();
    outfile.close();
    return 0;
}
