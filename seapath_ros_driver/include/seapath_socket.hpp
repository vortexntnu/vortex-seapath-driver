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

namespace seapath
{
    /**
     * @brief Class representing a socket for communication with a remote server.
     */
    class Socket
    {
    public:
        /**
         * @brief Constructor for Socket class.
         * @param UDP_IP The IP address of the remote server.
         * @param UDP_PORT The port number of the remote server.
         * @param shared_vector A reference to a vector for sharing data between threads.
         * @param mutex A reference to a mutex for synchronizing access to the shared vector.
         * @param packet_ready A reference to a boolean flag indicating if a packet is ready to be processed.
         * @param socket_connected A reference to a boolean flag indicating if the socket is connected.
         */
        Socket(std::string UDP_IP, u_int16_t UDP_PORT, std::vector<uint8_t> &shared_vector, std::mutex &mutex, bool &packet_ready, bool &socket_connected);

        /**
         * @brief Destructor for Socket class.
         */
        ~Socket();

        /**
         * @brief Closes the socket.
         */
        void close_socket();

        /**
         * @brief Creates a socket.
         */
        void create_socket();

        /**
         * @brief Connects to the remote server.
         */
        void connect_to_socket();

        /**
         * @brief Receives data from the remote server.
         */
        void receive_data();

        /**
         * @brief Socket file descriptor.
         */
        int client_socket_;

        /**
         * @brief IP address of the remote server.
         */
        std::string addr_;

        /**
         * @brief Port number of the remote server.
         */
        uint16_t port_;

        /**
         * @brief Buffer for storing received data.
         */
        uint8_t buffer_[1024];

        /**
         * @brief A reference to the shared vector for data exchange between threads.
         */
        std::vector<uint8_t> &shared_vector_;

        /**
         * @brief A reference to the mutex for synchronizing access to the shared vector.
         */
        std::mutex &mutex_;

        /**
         * @brief A reference to the flag indicating if a packet is ready to be processed.
         */
        bool &packet_ready_;

        /**
         * @brief A reference to the flag indicating if the socket is connected.
         */
        bool &socket_connected_;

        /**
         * @brief Server address structure for socket communication.
         */
        sockaddr_in servaddr_;

    private:
        
    };
} // namespace seapath
#endif // SEAPATH_SOCKET_H
