#include "seapath_ekf/ekf.hpp"

EKF::EKF() : X(6), P(6, 6), Q(6, 6), R(6, 6)
{
    X.setZero();
    P.setIdentity();
    Q.setIdentity() *= 0.01;
    R.setIdentity() *= 0.1;

    ROS_WARN("Waiting for MRU calibration...");
    ros::Duration(12.0).sleep(); // Hacky
    ROS_WARN("EKF ready!");

    imu_sub = nh.subscribe<sensor_msgs::Imu>("/sensor/mru", 10, &EKF::imuCallback, this);
    ahrs_sub = nh.subscribe("/estimator/ahrs", 1000, &EKF::updateAHRS, this);
    pose_sub = nh.subscribe("/sensor/seapath/pose/ned", 1000, &EKF::updatePose, this);
    twist_sub = nh.subscribe("/sensor/seapath/twist/ned", 1000, &EKF::updateTwist, this);

    odometry_pub = nh.advertise<nav_msgs::Odometry>("/estimator/pose", 10);

    R_pose = Eigen::MatrixXd::Identity(2, 2);
    R_pose(0, 0) = 0.1; 
    R_pose(1, 1) = 0.1;

    R_yaw = Eigen::MatrixXd::Identity(1, 1) * 0.01;

    R_twist = Eigen::MatrixXd::Identity(3, 3);
    R_twist(0, 0) = 0.05;
    R_twist(1, 1) = 0.05;
    R_twist(2, 2) = 0.01; 

    H_yaw = Eigen::MatrixXd::Zero(1, 6);
    H_yaw(0, 2) = 1; 

    H_pose = Eigen::MatrixXd::Zero(2, 6);
    H_pose(0, 0) = 1; 
    H_pose(1, 1) = 1; 

    H_twist = Eigen::MatrixXd::Zero(3, 6);
    H_twist(0, 3) = 1;
    H_twist(1, 4) = 1;
    H_twist(2, 5) = 1;


}

void EKF::predict(double dt)
{
    // Current state
    double x = X(0);
    double y = X(1);
    double yaw = X(2);
    double vx = X(3);
    double vy = X(4);
    double vyaw = X(5);

    // Update position using velocity and acceleration
    x += vx * dt + 0.5 * acceleration_in_world(0) * dt * dt;
    y += vy * dt + 0.5 * acceleration_in_world(1) * dt * dt;
    // For yaw, you can continue using vyaw or integrate if you have angular acceleration

    // Update velocity using acceleration
    vx += acceleration_in_world(0) * dt;
    vy += acceleration_in_world(1) * dt;
    // Again, for vyaw, use if you have angular acceleration

    // Store updated state
    X << x, y, yaw, vx, vy, vyaw;

    Eigen::MatrixXd F(6, 6);
    F.setIdentity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    X = F * X;
    P = F * P * F.transpose() + Q;
}


void EKF::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Extract linear acceleration
    Eigen::Vector3d accel_body(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // Transform acceleration from body to world frame
    tf::Quaternion orientation(X(2), 0, 0);  // Assuming your state has only yaw
    tf::Matrix3x3 rotation(orientation);

    Eigen::Matrix3d eigen_rotation;
    tf::matrixTFToEigen(rotation, eigen_rotation);
    
    acceleration_in_world = eigen_rotation * accel_body;
}

void EKF::updateAHRS(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Eigen::VectorXd y = Eigen::VectorXd(1);
    y(0) = yaw - X(2);

    Eigen::MatrixXd S = H_yaw * P * H_yaw.transpose() + R_yaw;
    Eigen::MatrixXd K = P * H_yaw.transpose() * S.inverse();

    X = X + K * y;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    P = (I - K * H_yaw) * P;

    // Normalize yaw to [-pi, pi]
    while (X(2) > M_PI) X(2) -= 2.0 * M_PI;
    while (X(2) < -M_PI) X(2) += 2.0 * M_PI;
}

void EKF::updatePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double x_meas = msg->pose.pose.position.x;
    double y_meas = msg->pose.pose.position.y;


    Eigen::VectorXd y = Eigen::VectorXd(2);
    y(0) = x_meas - X(0);
    y(1) = y_meas - X(1);

    Eigen::MatrixXd S = H_pose * P * H_pose.transpose() + R_pose;
    Eigen::MatrixXd K = P * H_pose.transpose() * S.inverse();

    X = X + K * y;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    P = (I - K * H_pose) * P;
}

void EKF::updateTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
    double vx_meas = msg->twist.twist.linear.x;
    double vy_meas = msg->twist.twist.linear.y;
    double vyaw_meas = msg->twist.twist.angular.z;


    Eigen::VectorXd y = Eigen::VectorXd(3);
    y(0) = vx_meas - X(3);
    y(1) = vy_meas - X(4);
    y(2) = vyaw_meas - X(5);

    Eigen::MatrixXd S = H_twist * P * H_twist.transpose() + R_twist;
    Eigen::MatrixXd K = P * H_twist.transpose() * S.inverse();
    X = X + K * y;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    P = (I - K * H_twist) * P;
}

void EKF::run()
{

    ros::Time current_time, last_time;
    last_time = ros::Time::now();

    ros::Rate rate(30);
    while (ros::ok())
    {
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        
        predict(dt);
        
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = X(0);
        odom.pose.pose.position.y = X(1);
        odom.pose.pose.position.z = 0;  // Assuming no vertical motion

        tf2::Quaternion quat;
        quat.setRPY(0, 0, X(2));  // Roll and pitch are set to 0

        odom.pose.pose.orientation.x = quat.getX();
        odom.pose.pose.orientation.y = quat.getY();
        odom.pose.pose.orientation.z = quat.getZ();
        odom.pose.pose.orientation.w = quat.getW();

        odom.twist.twist.linear.x = X(3);
        odom.twist.twist.linear.y = X(4);
        odom.twist.twist.angular.z = X(5);

        odometry_pub.publish(odom);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "body";
        transformStamped.transform.translation.x = X(0);
        transformStamped.transform.translation.y = X(1);
        transformStamped.transform.translation.z = 0;  // Assuming no vertical motion

        tf2::Quaternion q;
        q.setRPY(0, 0, X(2));  // Roll and pitch are 0
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster.sendTransform(transformStamped);
        
        last_time = current_time;

        ros::spinOnce();
        rate.sleep();
    }
}