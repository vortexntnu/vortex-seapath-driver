#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

ros::Publisher pose_pub_enu;
ros::Publisher twist_pub_enu;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    geometry_msgs::PoseWithCovarianceStamped pose_enu = *msg;

    // Swap x and y
    std::swap(pose_enu.pose.pose.position.x, pose_enu.pose.pose.position.y);

    // Invert z
    pose_enu.pose.pose.position.z = -pose_enu.pose.pose.position.z;

    // Convert orientation (Quaternion) from NED to ENU
    tf::Quaternion q_ned(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Quaternion q_rot = tf::createQuaternionFromRPY(M_PI, 0, M_PI);  // roll, pitch, yaw
    tf::Quaternion q_enu = q_ned * q_rot;

    pose_enu.pose.pose.orientation.x = q_enu.x();
    pose_enu.pose.pose.orientation.y = q_enu.y();
    pose_enu.pose.pose.orientation.z = q_enu.z();
    pose_enu.pose.pose.orientation.w = q_enu.w();

    pose_pub_enu.publish(pose_enu);
}

void twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) {
    geometry_msgs::TwistWithCovarianceStamped twist_enu = *msg;

    // Swap linear velocities (x and y) and invert z
    std::swap(twist_enu.twist.twist.linear.x, twist_enu.twist.twist.linear.y);
    twist_enu.twist.twist.linear.z = -twist_enu.twist.twist.linear.z;

    // Swap angular velocities (x and y) and invert z
    std::swap(twist_enu.twist.twist.angular.x, twist_enu.twist.twist.angular.y);
    twist_enu.twist.twist.angular.z = -twist_enu.twist.twist.angular.z;

    twist_pub_enu.publish(twist_enu);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ned_to_enu_converter");

    ros::NodeHandle nh;

    ros::Subscriber pose_sub_ned = nh.subscribe("/sensor/seapath/pose/ned", 1000, poseCallback);
    ros::Subscriber twist_sub_ned = nh.subscribe("/sensor/seapath/twist/ned", 1000, twistCallback);

    pose_pub_enu = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sensor/seapath/pose/enu", 1000);
    twist_pub_enu = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/sensor/seapath/twist/enu", 1000);

    ros::spin();

    return 0;
}
