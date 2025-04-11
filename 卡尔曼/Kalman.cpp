#include <iostream>
#include <eigen3/Eigen/Dense>


using namespace Eigen;
using namespace std;

class KalmanFilter {
public:
    KalmanFilter(double dt, const MatrixXd& A, const MatrixXd& H, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& P)
        : dt(dt), A(A), H(H), Q(Q), R(R), P(P), I(MatrixXd::Identity(A.rows(), A.rows())) {
        x = VectorXd::Zero(A.rows());
        // 默认情况下，不使用 B 和 u
        B = MatrixXd::Zero(A.rows(), 1);  // 如果没有 B，可以设置为零矩阵
        u = VectorXd::Zero(1);  // 如果没有 u，可以设置为零向量
    }

    void predict() {
        x = A * x + B * u;  // 预测时包括控制输入
        P = A * P * A.transpose() + Q;
    }

    void update(const VectorXd& z) {
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();
        x = x + K * (z - H * x);
        P = (I - K * H) * P;
    }

    VectorXd getState() const { return x; }

private:
    double dt;
    MatrixXd A, H, Q, R, P, I, B;
    VectorXd x, u;
};
