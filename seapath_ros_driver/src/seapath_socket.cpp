#include "seapath_socket.hpp"
#include "seapath_ros_driver.hpp"

namespace seapath
{

    Socket::Socket(std::string UDP_IP, u_int16_t UDP_PORT)
        : addr_(UDP_IP), port_(UDP_PORT), data_ready_(false)   {
    }

    void Socket::set_port(u_int16_t port)
    {
        port_ = port;
    }

    void Socket::set_ip(std::string ip)
    {
        addr_ = ip;
    }

    void Socket::create_socket()
    {
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ == -1)
        {
            throw std::runtime_error("Error creating socket");
        }

        memset(&socketaddr_, 0, sizeof(socketaddr_));

        // Set up server address information
        socketaddr_.sin_family = AF_INET;
        socketaddr_.sin_port = htons(port_);
        socketaddr_.sin_addr.s_addr = INADDR_ANY;
    }

    void Socket::bind_socket()
    {
        while (true)
        {
            std::cout << "[INFO] Attempting to bind socket to port " << port_ << std::endl;
            // Bind the socket with the server address
            int n = bind(socket_fd_, (struct sockaddr *)&socketaddr_, sizeof(socketaddr_));

            if (n < 0)
            {
                int timeout_delay = 5;
                std::cerr << "Error binding socket! Trying again in " << timeout_delay << " seconds" << std::endl;
                std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(timeout_delay));
                continue;
            }
            else
            {
                std::cout << "[INFO] Socket setup and ready to receive at port " << port_ << std::endl;
                break;
            }
        }
    }

    void Socket::receive_data()
    {
        struct sockaddr_in senderaddr_;
        socklen_t senderaddr_len = sizeof(senderaddr_);
        while (true)
        {
            // Receive data at address specified by socketaddr_
            int bytes_read = recvfrom(socket_fd_, buffer_.data(), KMB_SIZE, 0, (struct sockaddr *)&senderaddr_, &senderaddr_len);
            if (senderaddr_.sin_addr.s_addr != inet_addr(addr_.c_str()))
            {
                std::cerr << "Received data from unknown address. Ignoring." << std::endl;
                continue;
            }

            if (uint8_t(bytes_read) != KMB_SIZE) // If the packet is not 132 bytes, the socket is re-bound
            {
                socket_connected_ = false;
                close_socket();
                create_socket();
                bind_socket();
                continue;
            }
            std::unique_lock<std::mutex> lock(mutex_);
            socket_connected_ = true;
            data_ready_ = true;
            lock.unlock();
        }
    }

    bool Socket::socket_connected() const
    {   
        std::unique_lock<std::mutex> lock(mutex_);
        return socket_connected_;
    }

    bool Socket::get_data_status() const
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return data_ready_;
    }

    KMBinaryData Socket::get_kmbinary_data()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        data_ready_ = false;
        return parse_kmbinary_data(buffer_); // lock goes out of scope and is released automatically
    }

    KMBinaryData Socket::parse_kmbinary_data(std::array<uint8_t,KMB_SIZE> data)
    {

        KMBinaryData result;
        size_t offset = 0;

        /**
         * @brief Helper lambda to copy data and update the offset
         *
         */
        auto copyData = [&data, &offset](void *dest, size_t size)
        {
            std::memcpy(dest, data.data() + offset, size);
            offset += size;
        };
        
        
        copyData(result.start_id, 4);
        copyData(&result.dgm_length, 2);
        copyData(&result.dgm_version, 2);
        copyData(&result.utc_seconds, 4);
        copyData(&result.utc_nanoseconds, 4);
        copyData(&result.status, 4);
        copyData(&result.latitude, 8);
        copyData(&result.longitude, 8);
        copyData(&result.ellipsoid_height, 4);
        copyData(&result.roll, 4);
        copyData(&result.pitch, 4);
        copyData(&result.heading, 4);
        copyData(&result.heave, 4);
        copyData(&result.roll_rate, 4);
        copyData(&result.pitch_rate, 4);
        copyData(&result.yaw_rate, 4);
        copyData(&result.north_velocity, 4);
        copyData(&result.east_velocity, 4);
        copyData(&result.down_velocity, 4);
        copyData(&result.latitude_error, 4);
        copyData(&result.longitude_error, 4);
        copyData(&result.height_error, 4);
        copyData(&result.roll_error, 4);
        copyData(&result.pitch_error, 4);
        copyData(&result.heading_error, 4);
        copyData(&result.heave_error, 4);
        copyData(&result.north_acceleration, 4);
        copyData(&result.east_acceleration, 4);
        copyData(&result.down_acceleration, 4);
        copyData(&result.delayed_heave_utc_seconds, 4);
        copyData(&result.delayed_heave_utc_nanoseconds, 4);
        copyData(&result.delayed_heave, 4);
        
        return result;
    }

    void Socket::close_socket()
    {
        std::cout << "[INFO] Closing connection to server " << addr_ << " at port " << port_ << std::endl;
        close(socket_fd_);
    }

    Socket::~Socket()
    {
        std::cout << "[INFO] Closing connection to server " << addr_ << " at port " << port_ << std::endl;
        close(socket_fd_);
    }
} // namespace seapath