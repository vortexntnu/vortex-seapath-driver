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

/**
 * @brief SeaPathSocket class constructor that establishes a UDP socket connection.
 *
 * @param UDP_IP The IP address for UDP socket communication.
 * @param UDP_PORT The port number for UDP socket communication.
 */
    SeaPathSocket(const char* UDP_IP, const int UDP_PORT);
    ~SeaPathSocket();

/**
 * @brief Receives data from the UDP socket and returns it as a vector of uint8_t.
 *
 * @return A vector of uint8_t containing the received data, or an empty vector if the socket is disconnected.
 */
    std::vector<uint8_t> recieve_data();

/**
 * @brief A boolean the tells if the socket is connected or not.
 * It is used in get_diagnostic_message() and parse_kmbinary_data(std::vector<uint8_t> data).
 * Not completely sure if it is needed in every check in the constructor for the seaPathSocket.
 */
    bool socket_connected;

private:

/**
* @brief Socket file descriptor.
*/
    int sockfd;

/**
 * @brief Server and client address structures for socket communication.
 */
    struct sockaddr_in servaddr, cliaddr;

/**
 * @brief Timeout value for socket reads.
 */
    struct timeval read_timeout;
};
#endif //SEAPATH_SOCKET_H
