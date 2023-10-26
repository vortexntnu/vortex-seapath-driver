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
#include <vortex_msgs/msg/KMBinary.hpp>

#include <seapath_socket.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

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


class SeaPathRosDriver : public rclcpp::Node{
public: 
    SeaPathRosDriver(const char* UDP_IP, const int UDP_PORT, std::chrono::duration<double> timerPeriod);
    ~SeaPathRosDriver() = default;
    KMBinaryData getKMBinaryData();
    void publish(KMBinaryData data);
    geometry_msgs::msg::Point getOriginPublisher();

private:
    SeaPathSocket seaPathSocket;
    KMBinaryData parseKMBinaryData(std::vector<uint8_t> data);

    geometry_msgs::msg::PoseWithCovarianceStamped toPoseWithCovarianceStamped(const KMBinaryData& data);
    geometry_msgs::msg::TwistWithCovarianceStamped toTwistWithCovarianceStamped(const KMBinaryData& data);
    
    diagnostic_msgs::msg::DiagnosticStatus getDiagnosticPublisher();
    sensor_msgs::msg::NavSatFix getNavSatFixPublisher(const KMBinaryData& data); 
    vortex_msgs::msg::KMBinaryPublisher(const KMBinaryData& data);
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr origin_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnosticStatus_pub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub;


    std::pair<double, double> displacement_wgs84(double north, double east);
    double convert_dms_to_dd(double dms);
    void reset_origin(const KMBinaryData& data);

    double ORIGIN_N = -100;
    double ORIGIN_E = -100;
    double ORIGIN_H = -100;
    
    void timer_callback();
    rclcpp::TimerBase::SharedPtr _timer;
    const std::chrono::duration<double> timerPeriod;


    

};

#endif