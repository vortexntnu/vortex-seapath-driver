#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>  // for tf::createQuaternionFromRPY
#include <math.h>
#include <tf/tf.h>
#include <Eigen/Dense>

#include "seapath_ahrs/ahrs.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ahrs_node");
    AHRS node;
    ros::spin();
    return 0;
}

