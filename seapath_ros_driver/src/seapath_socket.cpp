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

    // Socket currently set up using INADDR_ANY, which means it will listen to all incoming traffic on the specified port.
    void Socket::create_socket()
    {
        client_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (client_socket_ == -1)
        {
            throw std::runtime_error("Error creating socket");
        }

        memset(&servaddr_, 0, sizeof(servaddr_));

        // Set up server address information
        servaddr_.sin_family = AF_INET;
        servaddr_.sin_port = htons(port_);
        // servaddr_.sin_addr.s_addr = inet_addr(addr_.c_str());
        
        // Socket failed to bind to the specified address, so it is set to INADDR_ANY
        servaddr_.sin_addr.s_addr = INADDR_ANY;
    }

    void Socket::bind_socket()
    {
        while (true)
        {
            std::cout << "[INFO] Client socket " << client_socket_ << std::endl;
            std::cout << "[INFO] Attempting to connect to server " << addr_ << " at port " << port_ << std::endl;
            // Bind the socket with the server address
            int n = bind(client_socket_, (struct sockaddr *)&servaddr_, sizeof(servaddr_));

            if (n < 0)
            {
                int timeout_delay = 5;
                std::cerr << "Error connecting to server! Trying again in " << timeout_delay << " seconds" << std::endl;
                std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(timeout_delay));
                continue;
            }
            else
            {
                std::cout << "[INFO] Socket setup and ready to receive from address " << addr_ << " at port " << port_ << std::endl;
                break;
            }
        }
    }

    void Socket::receive_data()
    {
        while (true)
        {
            // Receive data from the server
            int bytes_read = recv(client_socket_, buffer_, sizeof(buffer_), 0);

            if (bytes_read != 132) // If the packet is not 132 bytes, the socket is re-bound
            {
                socket_connected_ = false;
                bind_socket();
                continue;
            }
            socket_connected_ = true;
            std::vector<uint8_t> packet_data;

            for (int i = 0; i < bytes_read; i++)
            { // All data is stored in the vector
                packet_data.push_back(buffer_[i]);
            }

            parse_kmbinary_data(packet_data);
            packet_data.clear();

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
        return KMBinary_; // lock goes out of scope and is released automatically
    }

    void Socket::parse_kmbinary_data(std::vector<uint8_t> data)
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
        
        std::unique_lock<std::mutex> lock(mutex_);
        KMBinary_= result;
        data_ready_ = true;
        lock.unlock();
    }

    void Socket::close_socket()
    {
        std::cout << "[INFO] Closing connection to server " << addr_ << " at port " << port_ << std::endl;
        close(client_socket_);
    }

    Socket::~Socket()
    {
        std::cout << "[INFO] Closing connection to server " << addr_ << " at port " << port_ << std::endl;
        close(client_socket_);
    }
} // namespace seapath