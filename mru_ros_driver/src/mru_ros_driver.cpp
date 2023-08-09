#include "mru_ros_driver/mru_ros_driver.hpp"

MRURosDriver::MRURosDriver() {
    pub = nh.advertise<sensor_msgs::Imu>("/sensor/mru", 10);
    gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0;
    accel_bias_x = accel_bias_y = accel_bias_z = 0.0; // Initialize with calibrated values if available

    sum_angular_velocity = tf::Vector3(0.0, 0.0, 0.0);
    sum_linear_accel = tf::Vector3(0.0, 0.0, 0.0);

    number_of_calibration_samples = 1000;
    calibration_sample_counter = 0;

    is_initialized = false;

    ROS_WARN("Starting MRU calibration (10 seconds)! ENSURE THAT THE SENSOR DOES NOT MOVE DURING THIS TIME!");
}

// NED
void MRURosDriver::publishData(const SensorData& data) {
    if (!is_initialized)
    {
        // Accumulate measurements
        sum_angular_velocity += tf::Vector3(data.angular_rate_roll, data.angular_rate_pitch, data.angular_rate_yaw);
        sum_linear_accel += tf::Vector3(data.lin_acc_x, data.lin_acc_y, data.lin_acc_z);
        calibration_sample_counter++;

        // If enough samples are collected
        if (calibration_sample_counter >= number_of_calibration_samples)
        {
            gyro_bias_x = sum_angular_velocity.x() / number_of_calibration_samples;
            gyro_bias_y = sum_angular_velocity.y() / number_of_calibration_samples;
            gyro_bias_z = sum_angular_velocity.z() / number_of_calibration_samples;

            accel_bias_x = sum_linear_accel.x() / number_of_calibration_samples;
            accel_bias_y = sum_linear_accel.y() / number_of_calibration_samples;
            accel_bias_z = sum_linear_accel.z() / number_of_calibration_samples;

            is_initialized = true;

            ROS_INFO("MRU initialization complete!");
            ROS_INFO("Gyro Biases:");
            ROS_INFO("\t x: %f", gyro_bias_x);
            ROS_INFO("\t y: %f", gyro_bias_y);
            ROS_INFO("\t z: %f", gyro_bias_z);
            
            ROS_INFO("Acceleration Biases:");
            ROS_INFO("\t x: %f", accel_bias_x);
            ROS_INFO("\t y: %f", accel_bias_y);
            ROS_INFO("\t z: %f", accel_bias_z);
        }

        return; // Don't process this data for position/orientation calculations
    }
    sensor_msgs::Imu msg;

    tf2::Quaternion q;
    q.setRPY(data.roll, data.pitch, data.yaw);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "mru";

    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    double datasheet_rp_variance = 0.000000761;
    msg.orientation_covariance = {datasheet_rp_variance, 0, 0, 0, datasheet_rp_variance, 0, 0, 0, 0.01};

    msg.angular_velocity.x = data.angular_rate_roll - gyro_bias_x;
    msg.angular_velocity.y = data.angular_rate_pitch - gyro_bias_y;
    msg.angular_velocity.z = data.angular_rate_yaw - gyro_bias_z;
    double datasheet_avel_variance = 0.000000190;
    msg.angular_velocity_covariance = {datasheet_avel_variance, 0, 0, 0, datasheet_avel_variance, 0, 0, 0, datasheet_avel_variance};

    msg.linear_acceleration.x = data.lin_acc_x - accel_bias_x;
    msg.linear_acceleration.y = data.lin_acc_y - accel_bias_y;
    msg.linear_acceleration.z = data.lin_acc_z - accel_bias_z;
    double estimated_lin_covariance = 0.000001;
    msg.linear_acceleration_covariance = {estimated_lin_covariance, 0, 0, 0, estimated_lin_covariance, 0, 0, 0, estimated_lin_covariance};

    pub.publish(msg);
}
