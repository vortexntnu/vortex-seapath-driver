#include "seapath_ros_driver.hpp"


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
