#ifndef SEAPATH_ROS_DRIVER_H
#define SEAPATH_ROS_DRIVER_H

#include <iostream>
#include <sstream>
#include <cmath> 
#include <vector>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/transform_datatypes.h" 
#include "tf2/LinearMath/Quaternion.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "seapath_socket.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct KMBinaryData {
    char start_id[4] = "a";
    uint16_t dgm_length = 1;
    uint16_t dgm_version = 2;
    uint32_t utc_seconds = 3;
    uint32_t utc_nanoseconds = 4;
    uint32_t status = 5;
    double latitude = 6.0;
    double longitude = 7.0;
    float ellipsoid_height = 8;
    float roll = 9;
    float pitch= 10;
    float heading = 11;
    float heave = 12;
    float roll_rate = 13;
    float pitch_rate =14;
    float yaw_rate = 15;
    float north_velocity= 16;
    float east_velocity= 17;
    float down_velocity= 18;
    float latitude_error= 19;
    float longitude_error= 20;
    float height_error = 21;
    float roll_error = 22;
    float pitch_error = 23;
    float heading_error = 24;
    float heave_error = 25;
    float north_acceleration = 26;
    float east_acceleration = 27;
    float down_acceleration = 28;
    uint32_t delayed_heave_utc_seconds = 29;
    uint32_t delayed_heave_utc_nanoseconds = 30;
    float delayed_heave = 30;
};

class SeaPathRosDriver : public rclcpp::Node{
public: 
    SeaPathRosDriver(const char* UDP_IP, const int UDP_PORT, std::chrono::duration<double> timerPeriod);
    ~SeaPathRosDriver() = default;
    KMBinaryData getKMBinaryData();
    void publish(KMBinaryData data);
    

private:
    SeaPathSocket seaPathSocket;
    KMBinaryData parseKMBinaryData(std::vector<uint8_t> data);

    geometry_msgs::msg::PoseWithCovarianceStamped toPoseWithCovarianceStamped(const KMBinaryData& data);
    geometry_msgs::msg::TwistWithCovarianceStamped toTwistWithCovarianceStamped(const KMBinaryData& data);
    geometry_msgs::msg::Point getOriginPublisher();
    diagnostic_msgs::msg::DiagnosticStatus getDiagnosticPublisher();
    sensor_msgs::msg::NavSatFix getNavSatFixPublisher(const KMBinaryData& data); 

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr origin_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnosticStatus_pub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub;


    std::pair<double, double> displacement_wgs84(double north, double east);
    double convert_dms_to_dd(double dms);
    void resetOrigin(const KMBinaryData& data);

    double ORIGIN_N = -100;
    double ORIGIN_E = -100;
    double ORIGIN_H = -100;
    
    void timer_callback();
    rclcpp::TimerBase::SharedPtr _timer;
    const std::chrono::duration<double> timerPeriod;

};

#endif //SEAPATH_ROS_DRIVER_HSocket