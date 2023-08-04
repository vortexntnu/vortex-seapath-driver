#include "mru_ros_driver/mru_socket.hpp"

SensorData parseNMEA(std::string str) {
    std::istringstream ss(str);
    std::string token;
    
    // Skip the first three tokens
    for (int i = 0; i < 3; ++i) {
        std::getline(ss, token, ',');
    }

    double data[9];
    for (int i = 0; i < 9; ++i) {
        std::getline(ss, token, ',');
        data[i] = std::stod(token);
    }

    return SensorData(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
}

SensorData receiveNMEA(int sockfd, struct sockaddr_in& cliaddr) {
    char buffer[1024];
    socklen_t len = sizeof(cliaddr);

    int n = recvfrom(sockfd, (char *)buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr*)&cliaddr, &len);
    buffer[n] = '\0';  // ensure null-terminated string

    return parseNMEA(buffer);
}
