#include "seapath_socket.hpp"
#include "seapath_ros_driver.hpp"

namespace seapath {

Socket::Socket(std::string UDP_IP, u_int16_t UDP_PORT,std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& packet_ready, bool& socket_connected) 
: addr_ (UDP_IP), port_ (UDP_PORT), shared_vector_ (shared_vector), mutex_ (mutex), packet_ready_ (packet_ready),socket_connected_ (socket_connected) {
}

void Socket::create_socket(){
        client_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if(client_socket_ == -1){
            throw std::runtime_error("Error creating socket");
        }

    memset(&servaddr_, 0, sizeof(servaddr_));

    // Set up server address information
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(port_);
    // servaddr_.sin_addr.s_addr = inet_addr(addr_.c_str());
    servaddr_.sin_addr.s_addr = INADDR_ANY;

}

void Socket::connect_to_socket(){
    socket_connected_ = false;
    while(true){
        std::cout << "[INFO] Client socket " << client_socket_ << std::endl;
        std::cout << "[INFO] Attempting to connect to server " << addr_ << " at port " << port_ << std::endl;
        // Bind the socket with the server address
        int n = bind(client_socket_, (struct sockaddr *)&servaddr_, sizeof(servaddr_)); 
     
        if (n < 0) {
            int timeout_delay = 5;
            std::cerr << "Error connecting to server! Trying again in " << timeout_delay << " seconds" << std::endl;
            std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(timeout_delay));
            continue;
        }
        else{
            std::cout << "[INFO] Socket setup and ready to receive from address " << addr_ << " at port " << port_ << std::endl;
            break;
        }
        
    }
    
}

void Socket::receive_data() {
    std::vector<uint8_t> packet_data;
    while(true){

    std::cout << "[INFO] Waiting for data from server " << addr_ << " at port " << port_ << std::endl;

    //set the socket timout to x sec and x µsec
    // struct timeval tv;
    // tv.tv_sec = 3;
    // tv.tv_usec = 0;
    //setsockopt(client_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

        // Receive data from the server
        int bytes_read = recv(client_socket_, buffer_, sizeof(buffer_), 0); 

        std::cout << "Received " << bytes_read << " bytes!" << std::endl;


        if(bytes_read < 0){
            socket_connected_ = false;
            continue;
        }
        else{
            socket_connected_ = true;
            buffer_[bytes_read] = '\0'; // Make sure the buffer is getting ended (should not be necessary)
        }

        for (int i = 0; i < bytes_read; i++){ // All data is stored in the vector
            packet_data.push_back(buffer_[i]);
        }

        if(!packet_ready_){
            std::unique_lock<std::mutex> lock(mutex_); // Locks the shared vector (extra protection for thread-safe handling)
            shared_vector_ = packet_data;
            packet_ready_ = true;
            lock.unlock();
        }

        packet_data.clear(); // Resets the vector for a new packet-collection
    }   
}

void Socket::close_socket(){
    std::cout << "[INFO] Closing connection to server " << addr_ << " at port " << port_ << std::endl;
    close(client_socket_);
}

Socket::~Socket() {
    std::cout << "[INFO] Closing connection to server " << addr_ << " at port " << port_ << std::endl;
    close(client_socket_);
}
} // namespace seapath