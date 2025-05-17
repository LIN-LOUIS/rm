#ifndef SEND_IMAGE_HPP
#define SEND_IMAGE_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "message.hpp"

#define MAX_PER_PACKET 10218

class ImageSender {
public:
    ImageSender(const std::string& ip = "10.2.20.28", int port = 8000)
        : server_ip_(ip), server_port_(port) {}

    bool init() { return true; }

    // 发送指定路径的图像
    bool sendImage(const std::string& image_path) {
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            std::cerr << "[Sender] Failed to load image: " << image_path << std::endl;
            return false;
        }

        std::vector<uchar> encoded;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
        if (!cv::imencode(".jpg", image, encoded, params)) {
            std::cerr << "[Sender] Image encoding failed!" << std::endl;
            return false;
        }

        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            std::cerr << "[Sender] Failed to create socket!" << std::endl;
            return false;
        }

        sockaddr_in servaddr{};
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(server_port_);
        inet_pton(AF_INET, server_ip_.c_str(), &servaddr.sin_addr);

        if (connect(sockfd, (sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
            std::cerr << "[Sender] Connection to " << server_ip_ << " failed!" << std::endl;
            close(sockfd);
            return false;
        }

        size_t total_len = encoded.size();
        size_t sent_bytes = 0;
        unsigned int dataID = rand();

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

        close(sockfd);
        std::cout << "[Sender] Image sent successfully: " << image_path << std::endl;
        return true;
    }

private:
    std::string server_ip_;
    int server_port_;
};

#endif // SEND_IMAGE_HPP
