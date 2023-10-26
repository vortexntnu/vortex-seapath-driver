#include "seapath_socket.hpp"
#include "seapath_ros_driver.hpp"


SeaPathSocket::SeaPathSocket(const char* UDP_IP, const int UDP_PORT) {
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        socket_connected = false;
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(UDP_IP);
    servaddr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind failed");
        socket_connected = false;
        exit(EXIT_FAILURE);// Bind to the specified IP address
    }
    //if the socket was created successfully
    else{
        socket_connected = true;
    }

}

SeaPathSocket::~SeaPathSocket() {
    close(sockfd);
}

std::vector<uint8_t> SeaPathSocket::receiveData() {
    uint8_t buffer[1024];
    int len = sizeof(cliaddr);

    //set the socket timout to x sec and x µsec
    read_timeout.tv_sec  = 1;
    read_timeout.tv_usec = 0;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));
    int n = recvfrom(sockfd, buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *) &cliaddr, (socklen_t *)&len);

    //if socket is disconnected, n = -1. Returns empty vector, is not used anywhere if its dced
    if (n != -1){
        socket_connected = true;
        return std::vector<uint8_t>(buffer, buffer + n);
    }
    else{ 
        socket_connected = false;
        return std::vector<uint8_t>();
    }
}