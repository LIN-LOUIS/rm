#ifndef RECV_IMAGE_HPP
#define RECV_IMAGE_HPP

#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "message.hpp"

#define MAX_PACKET_SIZE sizeof(MessageBuffer)

class ImageReceiver {
public:
    ImageReceiver(const std::string& server_ip = "10.2.20.28", int port = 8000)
        : server_ip_(server_ip), server_port_(port), sockfd_(-1), image_count_(0) {}

    ~ImageReceiver() {
        if (sockfd_ != -1) close(sockfd_);
    }

    // 初始化 TCP 连接
    bool init() {
        sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ < 0) {
            std::cerr << "[Receiver] Failed to create socket!" << std::endl;
            return false;
        }

        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_);
        inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr);

        if (connect(sockfd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "[Receiver] Connection to " << server_ip_ << ":" << server_port_ << " failed!" << std::endl;
            return false;
        }

        std::cout << "[Receiver] Connected to " << server_ip_ << ":" << server_port_ << std::endl;
        return true;
    }

    // 接收图像并保存到目标目录
    bool receiveToFolder(const std::string& save_folder) {
        std::unordered_map<unsigned int, std::vector<uchar>> imageBuffers;
        std::unordered_map<unsigned int, size_t> totalLengths;

        while (true) {
            MessageBuffer msg;
            ssize_t bytes = recv(sockfd_, &msg, MAX_PACKET_SIZE, 0);
            if (bytes <= 0) break;

            if (msg.Start != START_SYMBOL || msg.End != END_SYMBOL || msg.MessageType != IMAGE_MSG) continue;

            auto& buffer = imageBuffers[msg.DataID];
            if (buffer.size() < msg.DataTotalLength)
                buffer.resize(msg.DataTotalLength);

            memcpy(&buffer[msg.Offset], msg.Data, msg.DataLength);
            totalLengths[msg.DataID] = msg.DataTotalLength;

            if (msg.Offset + msg.DataLength >= msg.DataTotalLength) {
                cv::Mat image = cv::imdecode(buffer, cv::IMREAD_COLOR);
                if (!image.empty()) {
                    std::string filename = save_folder + "/received_" + std::to_string(image_count_++) + ".jpg";
                    cv::imwrite(filename, image);
                    std::cout << "[Receiver] Saved: " << filename << std::endl;

                    // 可视化
                    cv::imshow("Received Image", image);
                    cv::waitKey(1);
                }

                imageBuffers.erase(msg.DataID);
                totalLengths.erase(msg.DataID);
            }
        }

        return true;
    }

private:
    std::string server_ip_;
    int server_port_;
    int sockfd_;
    int image_count_;
};

#endif // RECV_IMAGE_HPP
