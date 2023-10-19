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

class SeaPathSocket {
public:
    SeaPathSocket(const char* UDP_IP, const int UDP_PORT);
    ~SeaPathSocket();
    std::vector<uint8_t> receiveData();
    bool socketConnected;

private:
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    struct timeval read_timeout;
};
#endif //SEAPATH_SOCKET_H