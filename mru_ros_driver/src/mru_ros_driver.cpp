#include "mru_ros_driver/mru_ros_driver.hpp"

MRURosDriver::MRURosDriver() {
    pub = nh.advertise<sensor_msgs::Imu>("/sensor/mru", 10);
}

// NED
// void MRURosDriver::publishData(const SensorData& data) {
//     sensor_msgs::Imu msg;

//     tf2::Quaternion q;
//     q.setRPY(data.roll, data.pitch, data.yaw);

//     msg.header.stamp = ros::Time::now();
//     msg.header.frame_id = "imu0";

//     msg.orientation.x = q.x();
//     msg.orientation.y = q.y();
//     msg.orientation.z = q.z();
//     msg.orientation.w = q.w();
//     double datasheet_rp_variance = 0.000000761;
//     msg.orientation_covariance = {datasheet_rp_variance, 0, 0, 0, datasheet_rp_variance, 0, 0, 0, 0.01};

//     msg.angular_velocity.x = data.angular_rate_roll;
//     msg.angular_velocity.y = data.angular_rate_pitch;
//     msg.angular_velocity.z = data.angular_rate_yaw;
//     double datasheet_avel_variance = 0.000000190;
//     msg.angular_velocity_covariance = {datasheet_avel_variance, 0, 0, 0, datasheet_avel_variance, 0, 0, 0, datasheet_avel_variance};

//     msg.linear_acceleration.x = data.lin_acc_roll;
//     msg.linear_acceleration.y = data.lin_acc_pitch;
//     msg.linear_acceleration.z = data.lin_acc_yaw;
//     double estimated_lin_covariance = 0.000001;
//     msg.linear_acceleration_covariance = {estimated_lin_covariance, 0, 0, 0, estimated_lin_covariance, 0, 0, 0, estimated_lin_covariance};

//     pub.publish(msg);
// }

// ENU
void MRURosDriver::publishData(const SensorData& data) {
    sensor_msgs::Imu msg;

    // Convert the orientation from NED to ENU
    tf2::Quaternion q;
    q.setRPY(-data.pitch, data.roll, -data.yaw);  // ENU: roll->-pitch, pitch->roll, yaw->-yaw

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "mru";

    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    double datasheet_rp_variance = 0.000000761;
    msg.orientation_covariance = {datasheet_rp_variance, 0, 0, 0, datasheet_rp_variance, 0, 0, 0, 0.01};

    // Convert the angular velocity from NED to ENU
    msg.angular_velocity.x = -data.angular_rate_pitch;  // ENU: x->-y, y->x, z->-z
    msg.angular_velocity.y = data.angular_rate_roll;
    msg.angular_velocity.z = -data.angular_rate_yaw;
    double datasheet_avel_variance = 0.000000190;
    msg.angular_velocity_covariance = {datasheet_avel_variance, 0, 0, 0, datasheet_avel_variance, 0, 0, 0, datasheet_avel_variance};

    // Convert the linear acceleration from NED to ENU
    msg.linear_acceleration.x = -data.lin_acc_pitch;  // ENU: x->-y, y->x, z->-z
    msg.linear_acceleration.y = data.lin_acc_roll;
    msg.linear_acceleration.z = -data.lin_acc_yaw;
    double estimated_lin_covariance = 0.000001;
    msg.linear_acceleration_covariance = {estimated_lin_covariance, 0, 0, 0, estimated_lin_covariance, 0, 0, 0, estimated_lin_covariance};

    pub.publish(msg);
}
