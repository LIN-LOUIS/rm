#include "../include/recv_image.hpp"
#include "../include/send_image.hpp"
#include "../include/detector.hpp"
#include "../include/visualization.hpp"

int main() {
    std::cout << "Starting armor detection system..." << std::endl;

    // 初始化接收端并从通信接口保存图像
    ImageReceiver receiver;
    if (!receiver.init()) {
        std::cerr << "Failed to initialize image receiver!" << std::endl;
        return -1;
    }
    receiver.receiveToFolder("/home/lin/Desktop/大作业/armor_image_transfer/images");

    // 执行图像识别、PnP、卡尔曼滤波并输出预测中心轨迹 CSV
    Detector detector(
        "/home/lin/Desktop/大作业/armor_image_transfer/images",
        "/home/lin/Desktop/大作业/armor_image_transfer/result/trajectory_xy12.csv"
    );
    detector.run();

    // 可视化旋转中心轨迹并保存图像
    visualizeTrajectory(
        "/home/lin/Desktop/大作业/armor_image_transfer/result/trajectory_xy12.csv",
        "/home/lin/Desktop/大作业/armor_image_transfer/result/armor_rotation_xy_trajectory12.png"
    );

    // 发送轨迹图像回通信端
    ImageSender sender;
    if (!sender.init()) {
        std::cerr << "Failed to initialize image sender!" << std::endl;
        return -1;
    }
    sender.sendImage("/home/lin/Desktop/大作业/armor_image_transfer/result/armor_rotation_xy_trajectory12.png");


    return 0;
}