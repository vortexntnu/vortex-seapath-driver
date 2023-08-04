#include "seapath_gnss_ros_driver/seapath_ros_driver.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "seapath_ros_driver_node");
    ros::NodeHandle nh;

    std::string UDP_IP;
    int UDP_PORT;

    nh.param("UDP_IP", UDP_IP, std::string("0.0.0.0"));
    nh.param("UDP_PORT", UDP_PORT, 31420);

    SeaPathRosDriver seaPathRosDriver(nh, UDP_IP.c_str(), UDP_PORT);

    while (ros::ok()) {
        SeapathData seapath_data = seaPathRosDriver.getSeapathData();
        seaPathRosDriver.publish(seapath_data);
        ros::spinOnce();
    }

    return 0;
}
