#ifndef AHRS_HPP
#define AHRS_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>  // for tf::createQuaternionFromRPY
#include <tf/tf.h>
#include <Eigen/Dense>

tf::Vector3 bodyToWorld(const tf::Vector3& body_accel, const tf::Quaternion& orientation);

class AHRS
{
public:
    AHRS();

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    tf::Vector3 position;
    tf::Vector3 velocity;
    double roll, pitch, yaw;

    double last_time;
};

#endif // AHRS_HPP