#include "seapath_ros_driver.hpp"

geometry_msgs::msg::PoseWithCovarianceStamped SeaPathRosDriver::to_pose_with_covariance_stamped(const KMBinaryData& data) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    rclcpp::Time current_time;
    pose_msg.header.stamp = current_time = this->now();

    pose_msg.header.frame_id = "seapath/frame/pose"; 

    float north = data.latitude;
    float east = data.longitude;
    float height = data.ellipsoid_height;

    if(ORIGIN_N == -100 && ORIGIN_E == -100 && ORIGIN_H == -100) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Setting global starting origin to: " << north << ", " << east << "\n");
        ORIGIN_N = north;
        ORIGIN_E = east;
        ORIGIN_H = height;
    }

    auto xy = displacement_wgs84(north, east);

    pose_msg.pose.pose.position.x = xy.first;
    pose_msg.pose.pose.position.y = xy.second;
    pose_msg.pose.pose.position.z = height - ORIGIN_H;

    tf2::Quaternion q;
    q.setRPY(data.roll, data.pitch, data.heading);

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

geometry_msgs::msg::TwistWithCovarianceStamped SeaPathRosDriver::to_twist_with_covariance_stamped(const KMBinaryData& data) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    rclcpp::Time current_time;
    twist_msg.header.stamp = current_time = this->now();

    twist_msg.header.frame_id = "seapath/frame/twist";

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


geometry_msgs::msg::Point SeaPathRosDriver::get_origin_message(){
    
    geometry_msgs::msg::Point origin_msg;
    origin_msg.set__x(ORIGIN_N);
    origin_msg.set__y(ORIGIN_E);
    origin_msg.set__z(ORIGIN_H);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New Origin: " << "N:" << origin_msg.x << ", E:" << origin_msg.y << "H: " << origin_msg.z <<"\n");
    return origin_msg;
}


diagnostic_msgs::msg::DiagnosticStatus SeaPathRosDriver::get_diagnostic_message(){
    //check if it's properly connected to the socket
    diagnostic_msgs::msg::DiagnosticStatus diagnostic_msg;
    if(seaPathSocket.socket_connected == false){
        diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diagnostic_msg.name = "Diagnostic_connection_to_socket_status";
        diagnostic_msg.message = "Socket disconnected";
    }
    else if (seaPathSocket.socket_connected == true){
        diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diagnostic_msg.name = "Diagnostic_connection_to_socket_status";
        diagnostic_msg.message = "Socket connection is OK";
    }
    return diagnostic_msg;
}


sensor_msgs::msg::NavSatFix SeaPathRosDriver::get_navsatfix_message(const KMBinaryData& data){
    sensor_msgs::msg::NavSatFix nav_msg;
    rclcpp::Time current_time;
    auto pose_cov = to_pose_with_covariance_stamped(data);

    nav_msg.header.stamp = current_time = this->now();
    nav_msg.header.frame_id = "seapath/frame/NavSatFix";
    
    nav_msg.latitude = data.latitude;
    nav_msg.longitude = data.longitude;
    nav_msg.altitude = data.ellipsoid_height;


    for (int i = 0; i < 3; ++i){
        nav_msg.position_covariance[i + 0] = pose_cov.pose.covariance[i + 0];
        nav_msg.position_covariance[i + 3] = pose_cov.pose.covariance[i + 7];
        nav_msg.position_covariance[i + 6] = pose_cov.pose.covariance[i + 14];
    }

    return nav_msg;
}


vortex_msgs::msg::KMBinary SeaPathRosDriver::get_kmbinary_message(const KMBinaryData& data) {
    vortex_msgs::msg::KMBinary kmb_msg;

    kmb_msg.utc_seconds = data.utc_seconds;
    kmb_msg.utc_nanoseconds = data.utc_nanoseconds;

    kmb_msg.status = data.status;

    kmb_msg.latitude = data.latitude;
    kmb_msg.longitude = data.longitude;
    kmb_msg.ellipsoid_height = data.ellipsoid_height;

    kmb_msg.roll = data.roll;
    kmb_msg.pitch = data.pitch;
    kmb_msg.heading = data.heading;
    kmb_msg.heave = data.heave;

    kmb_msg.roll_rate = data.roll_rate;
    kmb_msg.pitch_rate = data.pitch_rate;
    kmb_msg.yaw_rate = data.yaw_rate;

    kmb_msg.north_velocity = data.north_velocity;
    kmb_msg.east_velocity = data.east_velocity;
    kmb_msg.down_velocity = data.down_velocity;

    kmb_msg.latitude_error = data.latitude_error;
    kmb_msg.longitude_error = data.longitude_error;
    kmb_msg.height_error = data.height_error;
    kmb_msg.roll_error = data.roll_error;
    kmb_msg.pitch_error = data.pitch_error;
    kmb_msg.heading_error = data.heading_error;
    kmb_msg.heave_error = data.heave_error;

    kmb_msg.north_acceleration = data.north_acceleration;
    kmb_msg.east_acceleration = data.east_acceleration;
    kmb_msg.down_acceleration = data.down_acceleration;

    kmb_msg.delayed_heave_utc_seconds = data.delayed_heave_utc_seconds;
    kmb_msg.delayed_heave_utc_nanoseconds = data.delayed_heave_utc_nanoseconds;
    kmb_msg.delayed_heave = data.delayed_heave;

    return kmb_msg;
} 

SeaPathRosDriver::SeaPathRosDriver(const char* UDP_IP, const int UDP_PORT, std::chrono::duration<double> timerPeriod) : Node("seapath_ros_driver_node"), seaPathSocket(UDP_IP, UDP_PORT), timerPeriod{timerPeriod}
{
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensor/seapath/pose/ned", 10);
    twist_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/sensor/seapath/twist/ned", 10);
    origin_pub = this->create_publisher<geometry_msgs::msg::Point>("/sensor/seapath/origin", 10);
    diagnosticStatus_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/sensor/seapath/diagnostic_msg", 10);
    nav_pub =  this->create_publisher<sensor_msgs::msg::NavSatFix>("/sensor/seapath/NavSatFix", 10);
    kmbinary_pub = this->create_publisher<vortex_msgs::msg::KMBinary>("/sensor/seapath/vortex_msgs",10);
    _timer = this->create_wall_timer(timerPeriod, std::bind(&SeaPathRosDriver::timer_callback, this));

}                

KMBinaryData SeaPathRosDriver::get_kmbinary_data() {
    std::vector<uint8_t> data = seaPathSocket.recieve_data();

    if (data.empty()){
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "WARNING: Receive data timed out");
    }

    return parse_kmbinary_data(data);
}

KMBinaryData SeaPathRosDriver::parse_kmbinary_data(std::vector<uint8_t> data) {
    KMBinaryData result;
    size_t offset = 0;

    // Helper lambda to copy data and update the offset
    auto copyData = [&data, &offset](void* dest, size_t size) {
        std::memcpy(dest, data.data() + offset, size);
        offset += size;
    };
    if(seaPathSocket.socket_connected){
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
    }

    return result;
}


void SeaPathRosDriver::publish(KMBinaryData data) {

    auto pose = to_pose_with_covariance_stamped(data);
    auto twist = to_twist_with_covariance_stamped(data);
    auto origin = get_origin_message();
    auto current_diagnostic = get_diagnostic_message();
    auto navSatFix = get_navsatfix_message(data);
    auto KMBinaryData = get_kmbinary_message(data);

    diagnosticStatus_pub->publish(current_diagnostic);

    if(current_diagnostic.level == diagnostic_msgs::msg::DiagnosticStatus::OK)
    {
        origin_pub->publish(origin);
        pose_pub->publish(pose);
        twist_pub->publish(twist);
        nav_pub->publish(navSatFix);
        kmbinary_pub->publish(KMBinaryData);
    }
}

std::pair<double, double> SeaPathRosDriver::displacement_wgs84(double north, double east) {
    double R = 6371.0;
    double m_per_deg_lat = R * 1000 * (M_PI / 180.0);
    double m_per_deg_lon = m_per_deg_lat * cos(ORIGIN_N * M_PI / 180.0);

    double displacement_north = (north - ORIGIN_N) * m_per_deg_lat;
    double displacement_east = (east - ORIGIN_E) * m_per_deg_lon;

    return {displacement_north, displacement_east};

}

void SeaPathRosDriver::reset_origin(const KMBinaryData& data){
    
    float north = data.latitude;
    float east = data.longitude;
    float height = data.ellipsoid_height;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Setting global origin to: " << north << ", " << east << "\n");
    ORIGIN_N = north;
    ORIGIN_E = east;
    ORIGIN_H = height;

}

double SeaPathRosDriver::convert_dms_to_dd(double dms) {
    double degrees = static_cast<int>(dms / 100);
    double minutes = dms - (degrees * 100);
    double dd = degrees + (minutes / 60);
    return round(dd * 1e6) / 1e6;
}

void SeaPathRosDriver::timer_callback(){
    KMBinaryData data = get_kmbinary_data();
    publish(data);
    }


std::ostream& operator<<(std::ostream& os, const KMBinaryData& data){
    os << "Start ID: " << data.start_id << std::endl;
    os << "Dgm Length: " << data.dgm_length << std::endl;
    os << "Dgm Version: " << data.dgm_version << std::endl;
    os << "UTC Seconds: " << data.utc_seconds << std::endl;
    os << "UTC Nanoseconds: " << data.utc_nanoseconds << std::endl;
    os << "Status: " << data.status << std::endl;
    os << "Latitude: " << data.latitude << std::endl;
    os << "Longitude: " << data.longitude << std::endl;
    os << "Ellipsoid Height: " << data.ellipsoid_height << std::endl;
    os << "Roll: " << data.roll << std::endl;
    os << "Pitch: " << data.pitch << std::endl;
    os << "Heading: " << data.heading << std::endl;
    os << "Heave: " << data.heave << std::endl;
    os << "Roll Rate: " << data.roll_rate << std::endl;
    os << "Pitch Rate: " << data.pitch_rate << std::endl;
    os << "Yaw Rate: " << data.yaw_rate << std::endl;
    os << "North Velocity: " << data.north_velocity << std::endl;
    os << "East Velocity: " << data.east_velocity << std::endl;
    os << "Down Velocity: " << data.down_velocity << std::endl;
    os << "Latitude Error: " << data.latitude_error << std::endl;
    os << "Longitude Error: " << data.longitude_error << std::endl;
    os << "Height Error: " << data.height_error << std::endl;
    os << "Roll Error: " << data.roll_error << std::endl;
    os << "Pitch Error: " << data.pitch_error << std::endl;
    os << "Heading Error: " << data.heading_error << std::endl;
    os << "Heave Error: " << data.heave_error << std::endl;
    os << "North Acceleration: " << data.north_acceleration << std::endl;
    os << "East Acceleration: " << data.east_acceleration << std::endl;
    os << "Down Acceleration: " << data.down_acceleration << std::endl;
    os << "Delayed Heave UTC Seconds: " << data.delayed_heave_utc_seconds << std::endl;
    os << "Delayed Heave UTC Nanoseconds: " << data.delayed_heave_utc_nanoseconds << std::endl;
    os << "Delayed Heave: " << data.delayed_heave << std::endl;
    return os;
}


void printKMBinaryData(const KMBinaryData& data) {
    std::cout << data;
}