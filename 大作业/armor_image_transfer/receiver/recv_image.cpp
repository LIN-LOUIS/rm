#include <iostream>
#include <opencv2/opencv.hpp>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "../include/message.hpp"

#define SERVER_IP "10.2.20.28"
#define SERVER_PORT 8000
#define MAX_PACKET_SIZE sizeof(MessageBuffer)

int main() {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket!" << std::endl;
        return -1;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    if (connect(sockfd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection to server failed!" << std::endl;
        return -1;
    }

    std::cout << "Connected to " << SERVER_IP << ":" << SERVER_PORT << std::endl;

    std::unordered_map<unsigned int, std::vector<uchar>> imageBuffers;
    std::unordered_map<unsigned int, size_t> totalLengths;
    int image_count = 0;

    while (true) {
        MessageBuffer msg;
        ssize_t bytes = recv(sockfd, &msg, MAX_PACKET_SIZE, 0);
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
                std::string filename = "../images/received_" + std::to_string(image_count++) + ".jpg";
                cv::imwrite(filename, image);
                std::cout << "Saved: " << filename << std::endl;
                cv::imshow("Received Image", image);
                cv::waitKey(1);
            }
            imageBuffers.erase(msg.DataID);
            totalLengths.erase(msg.DataID);
        }
    }

    close(sockfd);
    return 0;
}
