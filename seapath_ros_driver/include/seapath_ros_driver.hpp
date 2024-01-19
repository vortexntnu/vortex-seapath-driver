#ifndef SEAPATH_DRIVER_H
#define SEAPATH_DRIVER_H

#include <iostream>
#include <sstream>
#include <cmath> 
#include <vector>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <vortex_msgs/msg/km_binary.hpp>

#include "seapath_socket.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


/**
 * @brief The system data.
 * 
 */
struct KMBinaryData {
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
};


/**
 * @brief Out-stream override used when printing out the KMBinaryData
 * 
 * @param os The out-stream
 * @param data The systems data
 * @return std::ostream& Returens the out-stream filled with the systems data
 */
std::ostream& operator<<(std::ostream& os, const KMBinaryData& data);
void printKMBinaryData(const KMBinaryData& data);


/**
 * @brief The seapathdriver ros node.
 * 
 */
class SeaPathRosDriver : public rclcpp::Node{
public: 

/**
 * @brief Construct a new Sea Path Ros Driver object
 * 
 * @param UDP_IP The UDP IP-adress
 * @param UDP_PORT The UDP Port that is connected
 * @param timerPeriod Timer period publishing topics on the running node
 */
    SeaPathRosDriver(const char* UDP_IP, const int UDP_PORT, std::chrono::duration<double> timerPeriod);
    
/**
 * @brief Destroy the Sea Path Ros Driver object
 * 
 */
    ~SeaPathRosDriver() = default;

/**
 * @brief Get the kmbinary data object
 * 
 * @return KMBinaryData object
 */
    KMBinaryData get_kmbinary_data();

/**
 * @brief Publisher for KMBinary object
 * 
 * @param data The KMBinary object the is being published
 */
    void publish(KMBinaryData data);

/**
 * @brief Get the origin message object
 * 
 * @return geometry_msgs::msg::Point. Contains the location of the origin.
 */
    geometry_msgs::msg::Point get_origin_message();

private:
/**
 * @brief The socket we are connecting to.
 * 
 */
    SeaPathSocket seaPathSocket;

/**
 * @brief Parses the new data received from the socket
 * 
 * @param data Variable for storing the new data
 * @return KMBinaryData - the new data. Is only updated with new data IF the socket is connected. That is done by checking the socket.connected boolean.
 */
    KMBinaryData parse_kmbinary_data(std::vector<uint8_t> data);

/**
 * @brief Gets the covariance to the position as a message
 * 
 * @param data a KMBbinary object
 * @return geometry_msgs::msg::PoseWithCovarianceStamped - the message.
 */
    geometry_msgs::msg::PoseWithCovarianceStamped to_pose_with_covariance_stamped(const KMBinaryData& data);

/**
 * @brief Gets the covariance to the twist as a message
 * 
 * @param data a KMBbinary object
 * @return geometry_msgs::msg::TwistWithCovarianceStamped - the message
 */
    geometry_msgs::msg::TwistWithCovarianceStamped to_twist_with_covariance_stamped(const KMBinaryData& data);
    
/**
 * @brief Get the diagnostic message object. Uses the socket.connected boolean to make sure which meesage it should send.
 * 
 * @return diagnostic_msgs::msg::DiagnosticStatus - the message
 */
    diagnostic_msgs::msg::DiagnosticStatus get_diagnostic_message();

/**
 * @brief Get the navsatfix message object. Contains the position (longitude, latitude and height), and the covariance of the position. The dimensions of the covariance matrix to the navsatfix message and the position covariance message is not the same.
 * Therefore a for-loop is used to only get the first 4 elements from the position covariance message and copy them over to the navsatfix message. Very small covariance information is lost.
 * 
 * @param data The KMBinary object.
 * @return sensor_msgs::msg::NavSatFix - the message
 */
    sensor_msgs::msg::NavSatFix get_navsatfix_message(const KMBinaryData& data); 

/**
 * @brief Used for creating a message containing all the KMBinary data.
 * 
 * @param data The KMBinary data.
 * @return vortex_msgs::msg::KMBinary - the message
 */
    vortex_msgs::msg::KMBinary get_kmbinary_message(const KMBinaryData& data);

/**
 * @brief Get the diagnostic array object message.
 * 
 * @param diagnostic_msg The diagnostic message used for creating the diagnostic array.
 * @return diagnostic_msgs::msg::DiagnosticArray - the diagnostic array. Used for visualizing diagnostic status in programs like foxglove studio.
 */
    diagnostic_msgs::msg::DiagnosticArray get_diagnostic_array(diagnostic_msgs::msg::DiagnosticStatus diagnostic_msg);

/**
 * @brief Publishes the position covariance message
 * 
 */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;

/**
 * @brief Publishes the twist covariance message
 * 
 */
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub;

/**
 * @brief Publishes the origin position message
 * 
 */
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr origin_pub;

/**
 * @brief Publishes the diagnostic status message
 * 
 */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnosticStatus_pub;

/**
 * @brief Publishes the diagnostic array message
 * 
 */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticArray_pub

/**
 * @brief Publishes the navsatfix location message
 * 
 */
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub;

/**
 * @brief Publishes the KMBinary object as a message
 * 
 */
    rclcpp::Publisher<vortex_msgs::msg::KMBinary>::SharedPtr kmbinary_pub;

/**
 * @brief Calculates displacement in north and east directions from a reference position in the WGS84 coordinate system.
 *
 * @param north Northern coordinate in the WGS84 coordinate system.
 * @param east Eastern coordinate in the WGS84 coordinate system.
 * @return A std::pair<double, double> object containing the displacement in north and east directions.
 */
    std::pair<double, double> displacement_wgs84(double north, double east);

/**
 * @brief Converts a value from degrees, minutes, and seconds (DMS) format to decimal degrees (DD).
 *
 * @param dms Value in degrees, minutes, and seconds format.
 * @return The converted value in decimal degrees.
 */
    double convert_dms_to_dd(double dms);

/**
 * @brief Resets the global origin to the specified coordinates and height.
 *
 * @param data The KMBinaryData object containing latitude, longitude, and ellipsoid height.
 */
    void reset_origin(const KMBinaryData& data);

    double ORIGIN_N = -100;
    double ORIGIN_E = -100;
    double ORIGIN_H = -100;
    void timer_callback();

/**
 * @brief Timer callback function that retrieves KMBinaryData, then publishes it.
 */
    rclcpp::TimerBase::SharedPtr _timer;
    const std::chrono::duration<double> timerPeriod;
};

#endif //SEAPATH_DRIVER_H