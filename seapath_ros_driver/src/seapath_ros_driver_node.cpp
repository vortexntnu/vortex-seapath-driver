#include "seapath_ros_driver.hpp"


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    
    auto seaPathRosDriverNode = std::make_shared<seapath::Driver>();

    rclcpp::spin(seaPathRosDriverNode);
    rclcpp::shutdown();

    return 0;


}
