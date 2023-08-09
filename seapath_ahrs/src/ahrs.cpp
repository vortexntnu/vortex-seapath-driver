#include "seapath_ahrs/ahrs.hpp"

const double ALPHA = 0.0;  // Weight for gyro estimate. ALPHA = 0.0 means using angle measurements only

tf::Vector3 bodyToWorld(const tf::Vector3& body_accel, const tf::Quaternion& orientation)
{
    tf::Transform transform;
    transform.setRotation(orientation);
    return transform * body_accel;
}

AHRS::AHRS()
{
    // Initialize
    roll = pitch = yaw = 0.0;

    position = tf::Vector3(0.0, 0.0, 0.0);
    velocity = tf::Vector3(0.0, 0.0, 0.0);

    // ROS setup
    sub = nh.subscribe("/sensor/mru", 10, &AHRS::imuCallback, this);
    pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/estimator/ahrs", 10);
}

void AHRS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
        double current_time = msg->header.stamp.toSec();
        double dt = current_time - last_time;

        double qw = msg->orientation.w;
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;

        double roll_measure = atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
        double pitch_measure = asin(2.0*(qw*qy - qz*qx));
        double yaw_measure = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));

        double roll_dot = msg->angular_velocity.x;
        double pitch_dot = msg->angular_velocity.y;
        double yaw_dot = msg->angular_velocity.z;

        roll += roll_dot * dt;
        pitch += pitch_dot * dt;
        yaw += yaw_dot * dt;

        roll = fmod(roll + M_PI, 2.0 * M_PI) - M_PI;
        pitch = fmod(pitch + M_PI, 2.0 * M_PI) - M_PI;
        yaw = fmod(yaw + M_PI, 2.0 * M_PI) - M_PI;
        
        roll_measure = fmod(roll_measure + M_PI, 2.0 * M_PI) - M_PI;
        pitch_measure = fmod(pitch_measure + M_PI, 2.0 * M_PI) - M_PI;
        yaw_measure = fmod(yaw_measure + M_PI, 2.0 * M_PI) - M_PI;

        // Simple complementary filter
        roll = ALPHA * roll + (1.0 - ALPHA) * roll_measure;
        pitch = ALPHA * pitch + (1.0 - ALPHA) * pitch_measure;
        yaw = ALPHA * yaw + (1.0 - ALPHA) * yaw_measure;

        tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);

        tf::Vector3 body_accel(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
        tf::Vector3 world_accel = bodyToWorld(body_accel, tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

        velocity += world_accel * dt;
        position += velocity * dt;


        geometry_msgs::PoseWithCovarianceStamped pose_estimate;
        pose_estimate.header.stamp = ros::Time::now();
        pose_estimate.header.frame_id = "mru";
        pose_estimate.pose.pose.position.x = position[0];
        pose_estimate.pose.pose.position.y = position[1];
        pose_estimate.pose.pose.position.z = position[2];
        pose_estimate.pose.pose.orientation.x = q.x();
        pose_estimate.pose.pose.orientation.y = q.y();
        pose_estimate.pose.pose.orientation.z = q.z();
        pose_estimate.pose.pose.orientation.w = q.w();

        pub.publish(pose_estimate);

        last_time = current_time;
}