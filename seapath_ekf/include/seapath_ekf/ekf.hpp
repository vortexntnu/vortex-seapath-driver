#ifndef EKF_HPP
#define EKF_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <boost/bind.hpp>

class EKF {
private:
    Eigen::VectorXd X;
    Eigen::MatrixXd P, Q, R;

    ros::NodeHandle nh;
    ros::Subscriber ahrs_sub, pose_sub, twist_sub, imu_sub;
    ros::Publisher odometry_pub;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    Eigen::Vector3d acceleration_in_world;

    Eigen::MatrixXd R_pose;
    Eigen::MatrixXd R_yaw;
    Eigen::MatrixXd R_twist;

    Eigen::MatrixXd H_yaw;
    Eigen::MatrixXd H_pose;
    Eigen::MatrixXd H_twist;

public:
    EKF();
    void predict(double dt);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void updateAHRS(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void updatePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void updateTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
    void run();
};

#endif // EKF_HPP