#ifndef SEAPATH_ROS_DRIVER_H
#define SEAPATH_ROS_DRIVER_H

#include <iostream>
#include <sstream>
#include <math.h>
#include <vector>

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>


#include "seapath_gnss_ros_driver/seapath_socket.hpp"

struct SeapathData {
    double north = 0.0;
    double north_sigma = 0.0;

    double east = 0.0;
    double east_sigma = 0.0;

    double altitude = 0.0;
    double altitude_sigma = 0.0;

    double heading = 0.0;

    double sog_mps = 0.0;
    double cog_rad = 0.0;

    double rot = 0.0;

    double x_displacement = 0.0;
    double y_displacement = 0.0;
};

class SeaPathRosDriver {
public:
    SeaPathRosDriver(ros::NodeHandle nh, const char* UDP_IP, const int UDP_PORT);
    ~SeaPathRosDriver() = default;
    SeapathData getSeapathData();
    void publish(SeapathData data);

private:
    ros::NodeHandle nh;

    SeaPathSocket seaPathSocket;
    SeapathData parseNmeaData(std::string nmea_data);

    ros::Publisher nav_pub;
    ros::Publisher odom_pub;

    std::pair<double, double> displacement_wgs84(double north, double east);
    double convert_dms_to_dd(double dms);

    double ORIGIN_N = 0.0;
    double ORIGIN_E = 0.0;
};

#endif //SEAPATH_ROS_DRIVER_H
