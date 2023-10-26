#include <seapath_ros_driver.hpp>

void printKMBinaryData(const KMBinaryData& data) {
    std::cout << "Start ID: " << data.start_id << std::endl;
    std::cout << "Dgm Length: " << data.dgm_length << std::endl;
    std::cout << "Dgm Version: " << data.dgm_version << std::endl;
    std::cout << "UTC Seconds: " << data.utc_seconds << std::endl;
    std::cout << "UTC Nanoseconds: " << data.utc_nanoseconds << std::endl;
    std::cout << "Status: " << data.status << std::endl;
    std::cout << "Latitude: " << data.latitude << std::endl;
    std::cout << "Longitude: " << data.longitude << std::endl;
    std::cout << "Ellipsoid Height: " << data.ellipsoid_height << std::endl;
    std::cout << "Roll: " << data.roll << std::endl;
    std::cout << "Pitch: " << data.pitch << std::endl;
    std::cout << "Heading: " << data.heading << std::endl;
    std::cout << "Heave: " << data.heave << std::endl;
    std::cout << "Roll Rate: " << data.roll_rate << std::endl;
    std::cout << "Pitch Rate: " << data.pitch_rate << std::endl;
    std::cout << "Yaw Rate: " << data.yaw_rate << std::endl;
    std::cout << "North Velocity: " << data.north_velocity << std::endl;
    std::cout << "East Velocity: " << data.east_velocity << std::endl;
    std::cout << "Down Velocity: " << data.down_velocity << std::endl;
    std::cout << "Latitude Error: " << data.latitude_error << std::endl;
    std::cout << "Longitude Error: " << data.longitude_error << std::endl;
    std::cout << "Height Error: " << data.height_error << std::endl;
    std::cout << "Roll Error: " << data.roll_error << std::endl;
    std::cout << "Pitch Error: " << data.pitch_error << std::endl;
    std::cout << "Heading Error: " << data.heading_error << std::endl;
    std::cout << "Heave Error: " << data.heave_error << std::endl;
    std::cout << "North Acceleration: " << data.north_acceleration << std::endl;
    std::cout << "East Acceleration: " << data.east_acceleration << std::endl;
    std::cout << "Down Acceleration: " << data.down_acceleration << std::endl;
    std::cout << "Delayed Heave UTC Seconds: " << data.delayed_heave_utc_seconds << std::endl;
    std::cout << "Delayed Heave UTC Nanoseconds: " << data.delayed_heave_utc_nanoseconds << std::endl;
    std::cout << "Delayed Heave: " << data.delayed_heave << std::endl;
}


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    std::string UDP_IP;
    int UDP_PORT;
    
    auto seaPathRosDriverNode = std::make_shared<SeaPathRosDriver>(UDP_IP.c_str(), UDP_PORT, 100ms);

    seaPathRosDriverNode->get_parameter_or("UDP_IP", UDP_IP, std::string("0.0.0.0"));
    seaPathRosDriverNode->get_parameter_or("UDP_PORT", UDP_PORT, 31421);

    rclcpp::spin(seaPathRosDriverNode);
    rclcpp::shutdown();


    return 0;
}
