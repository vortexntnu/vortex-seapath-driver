#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

class EKF
{
public:
    EKF()
    {
        // Initialize state vector and matrices
        state.setZero();
        P.setIdentity() * 1e-6;
        Q <<    0.1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0.1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0.1, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.01, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0.01, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.01, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1e-4, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1e-4, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1e-4;

        R <<    1e-5, 0,
                0, 1e-5;
  


        // Start ROS subscribers and publishers
        imu_sub = nh.subscribe("/sensor/mru", 1000, &EKF::imuCallback, this);
        gnss_sub = nh.subscribe("/sensor/gnss/odom", 1000, &EKF::gnssCallback, this);
        state_pub = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 1000);

        state_pub_timer = nh.createTimer(ros::Duration(0.05), &EKF::publishState, this);

        last_time = ros::Time::now();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber gnss_sub;
    ros::Publisher state_pub;
    ros::Timer state_pub_timer;

    ros::Time last_time;

    // State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    Eigen::Matrix<double, 9, 1> state;

    // Uncertainty covariance
    Eigen::Matrix<double, 9, 9> P;

    // Process and measurement noise
    Eigen::Matrix<double, 9, 9> Q;
    Eigen::Matrix<double, 2, 2> R;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time).toSec();

        // Extract angular velocities and linear accelerations
        Eigen::Vector3d w(msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);
        Eigen::Vector3d a(msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z);

        // Update orientation using angular velocities
        Eigen::Matrix3d R = Eigen::AngleAxisd(w.norm() * dt, w.normalized()).toRotationMatrix();
        Eigen::Vector3d euler_angles = state.segment<3>(6);
        Eigen::Matrix3d orientation = (Eigen::AngleAxisd(euler_angles(2), Eigen::Vector3d::UnitZ())
                                    * Eigen::AngleAxisd(euler_angles(1), Eigen::Vector3d::UnitY())
                                    * Eigen::AngleAxisd(euler_angles(0), Eigen::Vector3d::UnitX())).toRotationMatrix();

        orientation = orientation * R;
        euler_angles = orientation.eulerAngles(2, 1, 0);
        state.segment<3>(6) = euler_angles;

        // Rotate linear acceleration to world frame
        Eigen::Vector3d world_acc = orientation.transpose() * a;

        // Update velocity using rotated acceleration
        state.segment<3>(3) += world_acc * dt;

        // Update position using updated velocity
        state.segment<3>(0) += state.segment<3>(3) * dt;

        // state(2) = 0.0; // zero Z height
        // state(5) = 0.0; // zero Z velocity

        // Prediction step of EKF
        Eigen::Matrix<double, 9, 9> F; // State transition matrix
        F.setIdentity();
        
        // Update the covariance
        P = F * P * F.transpose() + Q;

        last_time = current_time;
    }

    void gnssCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Update step of EKF

        Eigen::Matrix<double, 2, 1> z; // Measurement vector
        z << msg->pose.pose.position.x, msg->pose.pose.position.y;

        Eigen::Matrix<double, 2, 9> H; // Measurement matrix
        H.setZero();
        H(0, 0) = 1;
        H(1, 1) = 1;

        Eigen::Matrix<double, 2, 2> S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 9, 2> K = P * H.transpose() * S.inverse();

        // Update state and covariance
        state = state + K * (z - H * state);
        P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P;
    }

    void publishState(const ros::TimerEvent&)
    {
        nav_msgs::Odometry odom;

        // Set the frame_id and current time
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Populate the Odometry message with the state and covariance
        odom.pose.pose.position.x = state(0);
        odom.pose.pose.position.y = state(1);
        odom.pose.pose.position.z = state(2);
        odom.twist.twist.linear.x = state(3);
        odom.twist.twist.linear.y = state(4);
        odom.twist.twist.linear.z = state(5);

        // Convert roll, pitch, yaw to quaternion for orientation
        tf2::Quaternion q;
        q.setRPY(state(6), state(7), state(8));
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();   

        // Flatten the P matrix to a 1D array for the covariance
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                if(i < 3 && j < 3)
                {
                    // Covariance for x, y, z
                    odom.pose.covariance[6*i + j] = P(i, j);
                }
                else if(i >= 3 && j >= 3)
                {
                    // Covariance for roll, pitch, yaw
                    odom.pose.covariance[6*i + j] = P(i + 3, j + 3);
                }
            }
        }

        // Add covariance for twist
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                odom.twist.covariance[6*i + j] = P(i+3, j+3);
            }
        }

        state_pub.publish(odom);

        // Publish the transform between "odom" and "base_link"
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = state(0);
        transformStamped.transform.translation.y = state(1);
        transformStamped.transform.translation.z = state(2);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_node");
    EKF ekf;

    ros::spin();
    return 0;
}
