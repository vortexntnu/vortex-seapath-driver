#include "mru_ros_driver/mru_ros_driver.hpp"

MRURosDriver::MRURosDriver() {
    pub = nh.advertise<sensor_msgs::Imu>("/sensor/mru", 10);
}

void MRURosDriver::publishData(const SensorData& data) {
    sensor_msgs::Imu msg;

    tf2::Quaternion q;
    q.setRPY(data.roll, data.pitch, data.yaw);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "imu0";

    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();

    msg.angular_velocity.x = data.angular_rate_roll;
    msg.angular_velocity.y = data.angular_rate_pitch;
    msg.angular_velocity.z = data.angular_rate_yaw;

    msg.linear_acceleration.x = data.lin_acc_roll;
    msg.linear_acceleration.y = data.lin_acc_pitch;
    msg.linear_acceleration.z = data.lin_acc_yaw;

    pub.publish(msg);
}