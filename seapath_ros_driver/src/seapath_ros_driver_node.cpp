#include "seapath_ros_driver.hpp"


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    std::string UDP_IP;
    int UDP_PORT;
    
    auto seaPathRosDriverNode = std::make_shared<seapath::Driver>(UDP_IP.c_str(), UDP_PORT);

    seaPathRosDriverNode->get_parameter_or("UDP_IP", UDP_IP, std::string("10.0.1.10"));
    seaPathRosDriverNode->get_parameter_or("UDP_PORT", UDP_PORT, 31421);
    RCLCPP_INFO_STREAM(seaPathRosDriverNode->get_logger(), "UDP_IP: " << UDP_IP << " UDP_PORT: " << UDP_PORT);
    rclcpp::spin(seaPathRosDriverNode);
    rclcpp::shutdown();

    return 0;


}
