#include "seapath_socket.hpp"

SeaPathSocket::SeaPathSocket(const char* UDP_IP, const int UDP_PORT) {
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        socketConnected = false;
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(UDP_IP);
    servaddr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind failed");
        socketConnected = false;
        exit(EXIT_FAILURE);// Bind to the specified IP address
    }
    //if the socket was created successfully
    else{
        socketConnected = true;
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

