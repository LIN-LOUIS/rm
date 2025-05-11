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

#define MAX_PER_PACKET 10218

void sendImage(const std::string& ip, int port, const cv::Mat& image) {
    std::vector<uchar> encoded;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    cv::imencode(".jpg", image, encoded, params);

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &servaddr.sin_addr);

    if (connect(sockfd, (sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Connection failed!" << std::endl;
        return;
    }

    size_t total_len = encoded.size();
    size_t sent_bytes = 0;
    unsigned int dataID = 12345;

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
    close(sockfd);
}

int main() {
    cv::Mat img = cv::imread("../images/example.jpg");
    if (img.empty()) {
        std::cerr << "Failed to load image!" << std::endl;
        return -1;
    }

    sendImage("10.2.20.28", 8000, img);  // 使用目标IP与端口
    // 处理接收到图像后（模拟）发送回对方
// 此处建立新连接，将图像重新发回 sender（即远程服务器）
sendImage("10.2.20.28", 8000, img);  // 重新发送回对方（回环）
return 0;
}
