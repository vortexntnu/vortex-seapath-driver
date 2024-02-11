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
#include <chrono>
#include <mutex>

namespace seapath{
class Socket {
public:


   Socket(std::string UDP_IP, u_int16_t UDP_PORT,std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& packet_ready, bool& socket_connected);
   
    ~Socket();

    void close_socket();


    void create_socket();

    void connect_to_socket();


    void receive_data();

    /**
* @brief Socket file descriptor.
*/
    int client_socket_;

    std::string addr_;
    uint16_t port_;

    uint8_t buffer_[1024];

     std::vector<uint8_t>& shared_vector_;

    std::mutex& mutex_;

    bool& packet_ready_;

    bool& socket_connected_;


    sockaddr_in servaddr_;

private:




/**
 * @brief Server and client address structures for socket communication.
 */


};
} // namespace seapath
#endif //SEAPATH_SOCKET_H
