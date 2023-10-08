#include "seapath_ros_driver/include/seapath_ros_driver.hpp"

geometry_msgs::msg::PoseWithCovarianceStamped SeaPathRosDriver::toPoseWithCovarianceStamped(const KMBinaryData& data) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    rclcpp::Time current_time;
    pose_msg.header.stamp = current_time = nh.now();

    pose_msg.header.frame_id = "gnss"; 

    float north = data.latitude;
    float east = data.longitude;
    float height = data.ellipsoid_height;

    if(ORIGIN_N == -100 && ORIGIN_E == -100 && ORIGIN_H == -100) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Setting global origin to: " << north << ", " << east << "\n");
        ORIGIN_N = north;
        ORIGIN_E = east;
        ORIGIN_H = height;
        origin_pub->publish(ORIGIN_N, ORIGIN_E, ORIGIN_H);
    }

    auto xy = displacement_wgs84(north, east);

    pose_msg.pose.pose.position.x = xy.first;
    pose_msg.pose.pose.position.y = xy.second;
    pose_msg.pose.pose.position.z = height - ORIGIN_H;

    tf::Quaternion q = tf::createQuaternionFromRPY(data.roll, data.pitch, data.heading);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    pose_msg.pose.covariance[0] = data.longitude_error * data.longitude_error;
    pose_msg.pose.covariance[7] = data.latitude_error * data.latitude_error;
    pose_msg.pose.covariance[14] = data.height_error * data.height_error;
    pose_msg.pose.covariance[21] = data.roll_error * data.roll_error;
    pose_msg.pose.covariance[28] = data.pitch_error * data.pitch_error;
    pose_msg.pose.covariance[35] = data.heading_error * data.heading_error;

    return pose_msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped SeaPathRosDriver::toTwistWithCovarianceStamped(const KMBinaryData& data) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    rclcpp::Time current_time;
    twist_msg.header.stamp = current_time = nh.now();

    twist_msg.header.frame_id = "gnss";

    twist_msg.twist.twist.linear.x = data.north_velocity;
    twist_msg.twist.twist.linear.y = data.east_velocity;
    twist_msg.twist.twist.linear.z = data.down_velocity;

    twist_msg.twist.twist.angular.x = data.roll_rate;
    twist_msg.twist.twist.angular.y = data.pitch_rate;
    twist_msg.twist.twist.angular.z = data.yaw_rate;

    twist_msg.twist.covariance[0] = data.latitude_error * data.latitude_error;
    twist_msg.twist.covariance[7] = data.longitude_error * data.longitude_error;
    twist_msg.twist.covariance[14] = data.height_error * data.height_error; 


    // Temp hack to avoid inf in covariance
    if (data.heading_error > 10.0) {
        twist_msg.twist.covariance[35] = 10.0;
    } else {
        twist_msg.twist.covariance[35] = data.heading_error * data.heading_error;
    }
    twist_msg.twist.covariance[21] = data.roll_error * data.roll_error; 
    twist_msg.twist.covariance[28] = data.pitch_error * data.pitch_error;  

    return twist_msg;
}

SeaPathRosDriver::SeaPathRosDriver(rclcpp::Node nh, const char* UDP_IP, const int UDP_PORT) : seaPathSocket(UDP_IP, UDP_PORT) {
   
    pose_pub = nh.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensor/seapath/pose/ned", 10);
    twist_pub = nh.create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/sensor/seapath/twist/ned", 10);
    origin_pub = nh.create_publisher<geometry_msgs::msg::Point>("/sensor/origin", 10);
}

KMBinaryData SeaPathRosDriver::getKMBinaryData() {
    std::vector<uint8_t> data = seaPathSocket.receiveData();
    return parseKMBinaryData(data);
}

KMBinaryData SeaPathRosDriver::parseKMBinaryData(std::vector<uint8_t> data) {
    KMBinaryData result;
    size_t offset = 0;

    // Helper lambda to copy data and update the offset
    auto copyData = [&data, &offset](void* dest, size_t size) {
        std::memcpy(dest, data.data() + offset, size);
        offset += size;
    };

    copyData(result.start_id, 4);
    copyData(&result.dgm_length, 2);
    copyData(&result.dgm_version, 2);
    copyData(&result.utc_seconds, 4);
    copyData(&result.utc_nanoseconds, 4);
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


void SeaPathRosDriver::publish(KMBinaryData data) {
    auto pose = toPoseWithCovarianceStamped(data);
    auto twist = toTwistWithCovarianceStamped(data);

    pose_pub->publish(pose);
    twist_pub->publish(twist);

}

std::pair<double, double> SeaPathRosDriver::displacement_wgs84(double north, double east) {
    double R = 6371.0;
    double m_per_deg_lat = R * 1000 * (M_PI / 180.0);
    double m_per_deg_lon = m_per_deg_lat * cos(ORIGIN_N * M_PI / 180.0);

    double displacement_north = (north - ORIGIN_N) * m_per_deg_lat;
    double displacement_east = (east - ORIGIN_E) * m_per_deg_lon;

    return {displacement_north, displacement_east};

}

void SeaPathRosDriver::resetOrigin(){
    ORIGIN_E = data->longitude;
    ORIGIN_N = data.latitude;
}

double SeaPathRosDriver::convert_dms_to_dd(double dms) {
    double degrees = static_cast<int>(dms / 100);
    double minutes = dms - (degrees * 100);
    double dd = degrees + (minutes / 60);
    return round(dd * 1e6) / 1e6;
}