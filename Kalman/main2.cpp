#include <fstream>
#include <vector>
#include <sstream>
#include "Kalman.cpp"  // 包含 KalmanFilter 类的头文件
struct StockData {
    double day;
    double price;
};

std::vector<StockData> readStockData(const std::string& filename) {
    std::vector<StockData> data;
    std::ifstream file(filename);
    std::string line;

    // 跳过第一行标题
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string dayStr, priceStr;
        std::getline(ss, dayStr, ',');
        std::getline(ss, priceStr, ',');
        if (!dayStr.empty() && !priceStr.empty()) {
            data.push_back({ std::stod(dayStr), std::stod(priceStr) });
        }
    }
    return data;
}

int main() {
    auto data = readStockData("/home/lin/Desktop/卡尔曼波/data/stock_prices.csv");
    if (data.empty()) {
        cerr << "No data read from file!" << endl;
        return 1;
    }
    double dt = 1.0;
    MatrixXd A(2, 2);
    A << 1, dt,
         0, 1;
    
    MatrixXd H(1, 2);
    H << 1, 0;
    
    MatrixXd Q = MatrixXd::Identity(2, 2) * 1.0;  // 状态过程噪声
    MatrixXd R(1, 1);
    R << 10;  // 观测噪声
    
    MatrixXd P = MatrixXd::Identity(2, 2) * 1000.0;  // 初始协方差
    KalmanFilter kf(dt, A, H, Q, R, P);
    std::ofstream outFile("prediction_output.csv");
    outFile << "Day,Observed,Predicted\n";

    for (const auto& entry : data) {
        VectorXd z(1);
        z << entry.price;
        kf.predict();
        kf.update(z);
        outFile << entry.day << "," << entry.price << "," << kf.getState()(0) << "\n";
        
    }

    outFile.close();
    std::cout << "预测结果已保存到 prediction_output.csv" << std::endl;

    return 0;
}
