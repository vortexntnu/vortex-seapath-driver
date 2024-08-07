#ifndef SEAPATH_SOCKET_H
#define SEAPATH_SOCKET_H

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <sys/time.h>
#include <chrono>
#include <mutex>
#include <array>


namespace seapath
{
    struct KMBinaryData
    {
    char start_id[4];
    uint16_t dgm_length;
    uint16_t dgm_version;
    uint32_t utc_seconds;
    uint32_t utc_nanoseconds;
    uint32_t status;
    double latitude;
    double longitude;
    float ellipsoid_height;
    float roll;
    float pitch;
    float heading;
    float heave;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float north_velocity;
    float east_velocity;
    float down_velocity;
    float latitude_error;
    float longitude_error;
    float height_error;
    float roll_error;
    float pitch_error;
    float heading_error;
    float heave_error;
    float north_acceleration;
    float east_acceleration;
    float down_acceleration;
    uint32_t delayed_heave_utc_seconds;
    uint32_t delayed_heave_utc_nanoseconds;
    float delayed_heave;

    // Helper functions to check status bits
    bool isInvalidDataHorizontalPosVel() const { return status & (1 << 0); }
    bool isInvalidDataRollPitch() const { return status & (1 << 1); }
    bool isInvalidDataHeading() const { return status & (1 << 2); }
    bool isInvalidDataHeaveVertVel() const { return status & (1 << 3); }
    bool isInvalidDataAcceleration() const { return status & (1 << 4); }
    bool isInvalidDataDelayedHeave() const { return status & (1 << 5); }
    bool isAnyInvalidData() const { return status & 0x3F; }

    bool isReducedPerformanceHorizontalPosVel() const { return status & (1 << 16); }
    bool isReducedPerformanceRollPitch() const { return status & (1 << 17); }
    bool isReducedPerformanceHeading() const { return status & (1 << 18); }
    bool isReducedPerformanceHeaveVertVel() const { return status & (1 << 19); }
    bool isReducedPerformanceAcceleration() const { return status & (1 << 20); }
    bool isReducedPerformanceDelayedHeave() const { return status & (1 << 21); }
    bool isAnyReducedPerformance() const { return status & 0x3F0000; }

    enum class DiagnosticStatus {
        OK,    // No invalid or reduced performance flags are set
        REDUCED,  // Reduced performance flags are set, but no invalid flags are set
        INVALID  // Invalid flags are set
    };
    struct StatusFlags {
        DiagnosticStatus horizontal_pos_vel;
        DiagnosticStatus roll_pitch;
        DiagnosticStatus heading;
        DiagnosticStatus heave_vert_vel;
        DiagnosticStatus acceleration;
        DiagnosticStatus delayed_heave;
    };

    StatusFlags evaluateDiagnosticStatus() const {
        StatusFlags flags;

        flags.horizontal_pos_vel = determineStatus(isInvalidDataHorizontalPosVel(), isReducedPerformanceHorizontalPosVel());
        flags.roll_pitch = determineStatus(isInvalidDataRollPitch(), isReducedPerformanceRollPitch());
        flags.heading = determineStatus(isInvalidDataHeading(), isReducedPerformanceHeading());
        flags.heave_vert_vel = determineStatus(isInvalidDataHeaveVertVel(), isReducedPerformanceHeaveVertVel());
        flags.acceleration = determineStatus(isInvalidDataAcceleration(), isReducedPerformanceAcceleration());
        flags.delayed_heave = determineStatus(isInvalidDataDelayedHeave(), isReducedPerformanceDelayedHeave());

        return flags;
    }

    static DiagnosticStatus determineStatus(bool isInvalid, bool isReduced) {
        if (isInvalid) {
            return DiagnosticStatus::INVALID;
        } else if (isReduced) {
            return DiagnosticStatus::REDUCED;
        } else {
            return DiagnosticStatus::OK;
        }
    }

    DiagnosticStatus determineOverallStatus() const {
        if (isAnyInvalidData()) {
            return DiagnosticStatus::INVALID;
        } else if (isAnyReducedPerformance()) {
            return DiagnosticStatus::REDUCED;
        } else {
            return DiagnosticStatus::OK;
        }
    }

    static std::string status_to_string(KMBinaryData::DiagnosticStatus status) {
        switch (status)
        {
        case KMBinaryData::DiagnosticStatus::INVALID:
            return "INVALID";
        case KMBinaryData::DiagnosticStatus::REDUCED:
            return "REDUCED";
        case KMBinaryData::DiagnosticStatus::OK:
            return "OK";
        }
        return "UNKNOWN";
    }

    bool status_not_invalid() const {
        return determineOverallStatus() != DiagnosticStatus::INVALID;
    }

    bool status_ok() const {
        return determineOverallStatus() == DiagnosticStatus::OK;
    }

    };

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
         */
        Socket(std::string UDP_IP, u_int16_t UDP_PORT);

        Socket() { 
        }

        /**
         * @brief Destructor for Socket class.
         */
        ~Socket();

        /**
         * @brief Closes the socket.
         */
        void close_socket();

        void set_port(u_int16_t port);

        void set_ip(std::string ip);


        /**
         * @brief Creates a socket.
         */
        void create_socket();

        /**
         * @brief Connects to the remote server.
         */
        void bind_socket();

        /**
         * @brief Receives data from the remote server.
         */
        void receive_data();

        static constexpr size_t KMB_SIZE = 132;

        KMBinaryData parse_kmbinary_data(std::array<uint8_t,KMB_SIZE> data);

        KMBinaryData get_kmbinary_data();

        bool get_data_status() const;

        bool socket_connected() const;



        KMBinaryData KMBinary_;

        /**
         * @brief Socket file descriptor.
         */
        int socket_fd_;

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
        std::array<uint8_t, KMB_SIZE> buffer_;

        /**
         * @brief A reference to the mutex for synchronizing access to the shared vector.
         */
        mutable std::mutex mutex_;

        /**
         * @brief Server address structure for socket communication.
         */
        sockaddr_in socketaddr_;

        bool data_ready_;

        bool socket_connected_ = false;

    private:
        
    };
} // namespace seapath
#endif // SEAPATH_SOCKET_H
