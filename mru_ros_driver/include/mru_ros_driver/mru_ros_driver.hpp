#ifndef MRU_ROS_DRIVER_H
#define MRU_ROS_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

#include "mru_ros_driver/mru_socket.hpp"

class MRURosDriver {
public:
    MRURosDriver();
    void publishData(const SensorData& data);

private:
    ros::Publisher pub;
    ros::NodeHandle nh;
};

#endif // MRU_ROS_DRIVER_H