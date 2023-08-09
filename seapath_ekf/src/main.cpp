#include <ros/ros.h>

#include "seapath_ekf/ekf.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_node");
    EKF ekf_node;
    ekf_node.run();
    return 0;
}

