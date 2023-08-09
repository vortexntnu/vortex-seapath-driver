#ifndef MRU_ROS_DRIVER_H
#define MRU_ROS_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include "mru_ros_driver/mru_socket.hpp"

class MRURosDriver {
public:
    MRURosDriver();
    void publishData(const SensorData& data);

private:
    ros::Publisher pub;
    ros::NodeHandle nh;

    double accel_bias_x, accel_bias_y, accel_bias_z;
    double gyro_bias_x, gyro_bias_y, gyro_bias_z;

    bool is_initialized;
    tf::Vector3 sum_angular_velocity;
    tf::Vector3 sum_linear_accel;
    int number_of_calibration_samples;
    int calibration_sample_counter;
};

#endif // MRU_ROS_DRIVER_H