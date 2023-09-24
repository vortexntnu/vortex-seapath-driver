#ifndef SEAPATH_ROS_DRIVER_H
#define SEAPATH_ROS_DRIVER_H

#include <iostream>
#include <sstream>
#include <cmath> 
#include <vector>

#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/NavSatFix.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "geometry_msgs/msg/TwistWithCovarianceStamped.hpp"
#include "geometry_msgs/msg/PoseWithCovarianceStamped.hpp"
#include "tf2/transform_datatypes.h" 

#include "seapath_gnss_ros_driver/seapath_socket.hpp"






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

class SeaPathRosDriver {
public:
    SeaPathRosDriver(rclcpp::Node nh, const char* UDP_IP, const int UDP_PORT);
    ~SeaPathRosDriver() = default;
    KMBinaryData getKMBinaryData();
    void publish(KMBinaryData data);

private:
    rclcpp::Node nh;

    SeaPathSocket seaPathSocket;
    KMBinaryData parseKMBinaryData(std::vector<uint8_t> data);
    geometry_msgs::PoseWithCovarianceStamped toPoseWithCovarianceStamped(const KMBinaryData& data);
    geometry_msgs::TwistWithCovarianceStamped toTwistWithCovarianceStamped(const KMBinaryData& data);

    rclcpp::Publisher nav_pub;
    rclcpp::Publisher pose_pub;
    rclcpp::Publisher twist_pub;

    std::pair<double, double> displacement_wgs84(double north, double east);
    double convert_dms_to_dd(double dms);

    double ORIGIN_N = -100;
    double ORIGIN_E = -100;
    double ORIGIN_H = -100;
};

#endif //SEAPATH_ROS_DRIVER_H