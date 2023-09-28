#include "seapath_gnss_ros_driver/seapath_socket.hpp"

SeaPathSocket::SeaPathSocket(const char* UDP_IP, const int UDP_PORT) {
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
}

SeaPathSocket::~SeaPathSocket() {
    close(sockfd);
}

std::vector<uint8_t> SeaPathSocket::receiveData() {
    uint8_t buffer[1024];
    int len = sizeof(cliaddr);
    int n = recvfrom(sockfd, buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *) &cliaddr, (socklen_t *)&len);
    return std::vector<uint8_t>(buffer, buffer + n);
}