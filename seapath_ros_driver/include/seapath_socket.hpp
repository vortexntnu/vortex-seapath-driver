#ifndef SEAPATH_SOCKET_H
#define SEAPATH_SOCKET_H

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <sys/time.h>

class seapath_socket {
public:
    seapath_socket(const char* UDP_IP, const int UDP_PORT);
    ~seapath_socket();
    std::vector<uint8_t> receiveData();
    bool socket_connected;

private:
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    struct timeval read_timeout;
};
#endif //seapath_socket_h
