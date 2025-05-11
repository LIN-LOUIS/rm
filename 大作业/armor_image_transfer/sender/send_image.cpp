// 文件：sender/send_image.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "../include/message.hpp"

#define MAX_PER_PACKET 10218  // 每个数据包最大负载（与协议定义一致）

// 发送图像函数：将图像压缩成JPEG格式后，按协议分包并通过TCP发送
void sendImage(const std::string& ip, int port, const cv::Mat& image) {
    std::vector<uchar> encoded;  // 存储压缩图像字节流
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};  // 压缩质量
    cv::imencode(".jpg", image, encoded, params);  // 将图像压缩成JPEG格式

    // 创建 TCP 套接字
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &servaddr.sin_addr);

    // 尝试连接服务器
    if (connect(sockfd, (sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Connection failed!" << std::endl;
        return;
    }

    size_t total_len = encoded.size();  // 总长度
    size_t sent_bytes = 0;              // 已发送字节数
    unsigned int dataID = 12345;        // 可设置唯一ID以区分多个图像流

    // 按块发送图像数据（模拟协议封装）
    while (sent_bytes < total_len) {
        MessageBuffer msg{};
        msg.Start = START_SYMBOL;
        msg.MessageType = IMAGE_MSG;
        msg.DataID = dataID;
        msg.DataTotalLength = total_len;
        msg.Offset = sent_bytes;

        int chunk = std::min((size_t)MAX_PER_PACKET, total_len - sent_bytes);
        msg.DataLength = chunk;
        std::memcpy(msg.Data, encoded.data() + sent_bytes, chunk);
        msg.End = END_SYMBOL;

        send(sockfd, &msg, sizeof(MessageBuffer), 0);
        sent_bytes += chunk;
    }

    std::cout << "Image sent in " << (sent_bytes / MAX_PER_PACKET + 1) << " packet(s)." << std::endl;
    close(sockfd);  // 关闭连接
}

int main() {
    // 读取图像
    cv::Mat img = cv::imread("../images/example.jpg");
    if (img.empty()) {
        std::cerr << "Failed to load image!" << std::endl;
        return -1;
    }

    // 第一次发送图像：原始图像 → 远程服务器
    sendImage("10.2.20.28", 8000, img);  // 使用目标IP与端口发送图像
    // 第二次发送图像：处理后图像 → 再次发回服务器（模拟应答）
    sendImage("10.2.20.28", 8000, img);  // 再次发送，模拟"处理后返回"
    
    return 0;
}
