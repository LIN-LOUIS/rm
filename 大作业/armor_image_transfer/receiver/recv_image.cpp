#include <iostream>
#include <opencv2/opencv.hpp>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "../include/message.hpp"

// 设置远程服务器 IP 和端口
#define SERVER_IP "10.2.20.28"
#define SERVER_PORT 8000

// 单个数据包最大长度（与协议一致）
#define MAX_PACKET_SIZE sizeof(MessageBuffer)

int main() {
    // 创建 TCP 套接字
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket!" << std::endl;
        return -1;
    }

    // 配置服务器地址结构
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    // 连接到服务器
    if (connect(sockfd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection to server failed!" << std::endl;
        return -1;
    }

    std::cout << "Connected to " << SERVER_IP << ":" << SERVER_PORT << std::endl;

    // 存储每个图像的数据包缓冲区，按 DataID 区分
    std::unordered_map<unsigned int, std::vector<uchar>> imageBuffers;
    std::unordered_map<unsigned int, size_t> totalLengths;
    int image_count = 0;

    // 接收循环
    while (true) {
        MessageBuffer msg;
        ssize_t bytes = recv(sockfd, &msg, MAX_PACKET_SIZE, 0);
        if (bytes <= 0) break;  // 接收失败或连接关闭

        // 协议完整性校验
        if (msg.Start != START_SYMBOL || msg.End != END_SYMBOL || msg.MessageType != IMAGE_MSG) continue;

        // 获取对应 ID 的缓冲区
        auto& buffer = imageBuffers[msg.DataID];

        // 初始化缓冲区大小
        if (buffer.size() < msg.DataTotalLength)
            buffer.resize(msg.DataTotalLength);

        // 将数据段写入正确位置
        memcpy(&buffer[msg.Offset], msg.Data, msg.DataLength);
        totalLengths[msg.DataID] = msg.DataTotalLength;

        // 判断是否是最后一个包（简化方式）
        if (msg.Offset + msg.DataLength >= msg.DataTotalLength) {
            // 解码图像
            cv::Mat image = cv::imdecode(buffer, cv::IMREAD_COLOR);
            if (!image.empty()) {
                // 保存图像
                std::string filename = "../images/received_" + std::to_string(image_count++) + ".jpg";
                cv::imwrite(filename, image);
                std::cout << "Saved: " << filename << std::endl;

                // 显示图像
                cv::imshow("Received Image", image);
                cv::waitKey(1);
            }

            // 清除数据缓存
            imageBuffers.erase(msg.DataID);
            totalLengths.erase(msg.DataID);
        }
    }

    // 关闭 socket
    close(sockfd);
    return 0;
}
