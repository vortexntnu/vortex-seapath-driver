#include <seapath_ros_driver.hpp>

namespace seapath
{

    Driver::Driver() : Node("seapath_ros_driver_node")
    {
        // Define the quality of service profile for publisher and subscriber
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
        rmw_qos_profile_t qos_profile_transient_local = rmw_qos_profile_parameters;
        qos_profile_transient_local.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        auto qos_transient_local = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_transient_local.history, 1), qos_profile_transient_local);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/seapath/odom/ned", qos_sensor_data);
        diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", qos_transient_local);
        origin_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/origin/navsatfix", qos_transient_local);
        nav_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/seapath/navsatfix", qos_sensor_data);
        kmbinary_pub_ = this->create_publisher<vortex_msgs::msg::KMBinary>("/seapath/KMBinary", qos_sensor_data);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // Initialize the reset origin service server
        reset_origin_service_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_origin", std::bind(&Driver::reset_origin_callback, this, std::placeholders::_1, std::placeholders::_2));

        declare_parameter<std::string>("UDP_IP", "10.0.1.10");
        declare_parameter<u_int16_t>("UDP_PORT", 31421);
    
        UDP_IP_ = get_parameter("UDP_IP").as_string();
        UDP_PORT_ = get_parameter("UDP_PORT").as_int();

        socket_.set_ip(UDP_IP_);
        socket_.set_port(UDP_PORT_);
    
        std::thread(&Driver::setup_socket, this).detach();
        std::thread(&Driver::initial_setup, this).detach();

    }

    void Driver::setup_socket()
    {
        socket_.create_socket();
        socket_.bind_socket();
        socket_.receive_data();
    }

    void Driver::reset_origin()
    {
         // Block until a valid data message is received
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Waiting for valid data to set origin\n");
        while (!origin_set_) {
            if (socket_.get_data_status()) {
                data_ = socket_.get_kmbinary_data();
                if(get_diagnostic_array().status[0].level != diagnostic_msgs::msg::DiagnosticStatus::OK){ // Don't set origin if data not accurate
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                set_origin();
                time_ = this->get_clock()->now();
                origin_pub_->publish(get_navsatfix_message());
                publish_static_tf();
                driver_publisher();

                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void Driver::initial_setup()
    {
        reset_origin();
        // Start the timer callback loop after the origin is set
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Driver::timer_callback, this));
    }

    void Driver::timer_callback()
    {
       if(socket_.get_data_status()){
           data_ = socket_.get_kmbinary_data();
           time_ = this->get_clock()->now();
           driver_publisher();
    }
    }

    void Driver::reset_origin_callback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if(!socket_.socket_connected()){
            response->success = false;
            response->message = "Cannot reset origin when socket is disconnected";
            return;
        }

        if(timer_){
        timer_.reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Wait for timer to finish callback
        // Reset the origin coordinates
        origin_lat_ = 0.0;
        origin_lon_ = 0.0;
        origin_h_ = 0.0;
        origin_set_ = false;

        reset_origin();
        
        // Respond to the request
        response->success = true;
        response->message = "Origin reset successfully";

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Setting global origin to: " << origin_lat_ << ", " << origin_lon_ << ", " << origin_h_ << "\n");

        // Start the timer callback loop after the origin is set
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Driver::timer_callback, this));
        return;
    }

    void Driver::publish_static_tf(){
        // Setup the transform
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = time_;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "world_fox";

        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        // NED to SEU
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 1.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 0.0;

        // Broadcast the static transform
        static_tf_broadcaster_->sendTransform(transformStamped);

    }

    void Driver::driver_publisher() const
    {
        kmbinary_pub_->publish(get_kmbinary_message());
        diagnostic_pub_->publish(get_diagnostic_array());
        nav_pub_->publish(get_navsatfix_message());
        odom_pub_->publish(get_odometry_message());
        publish_dyn_tf();  
    }

    void Driver::set_origin()
    {
        origin_lat_ = data_.latitude;
        origin_lon_ = data_.longitude;
        origin_h_ = data_.ellipsoid_height;
        origin_set_ = true;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Setting global origin to: " << origin_lat_ << ", " << origin_lon_ << ", " << origin_h_ << "\n");
    }


    diagnostic_msgs::msg::DiagnosticArray Driver::get_diagnostic_array() const
    {
        diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_msg;
        diagnostic_msg.name = "Diagnostic_seapath_status";
        diagnostic_array.header.stamp = time_;

        // Check socket connection first
        if (!socket_.socket_connected())
        {
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diagnostic_msg.message = "Socket disconnected";
            diagnostic_array.status.push_back(diagnostic_msg);
            return diagnostic_array;
        }

        // Using status bit functions to check errors
        bool position_error = data_.isInvalidDataHorizontalPosVel() || data_.isReducedPerformanceHorizontalPosVel();
        bool orientation_error = data_.isInvalidDataRollPitch() || data_.isInvalidDataHeading() || data_.isReducedPerformanceRollPitch() || data_.isReducedPerformanceHeading();

        // Create KeyValue pairs for the diagnostic message
        diagnostic_msgs::msg::KeyValue kv1, kv2, kv3, kv4, kv5, kv6;

        kv1.key = "Latitude Error";
        kv1.value = std::to_string(data_.latitude_error);

        kv2.key = "Longitude Error";
        kv2.value = std::to_string(data_.longitude_error);

        kv3.key = "Height Error";
        kv3.value = std::to_string(data_.height_error);

        kv4.key = "Roll Error";
        kv4.value = std::to_string(data_.roll_error);

        kv5.key = "Pitch Error";
        kv5.value = std::to_string(data_.pitch_error);

        kv6.key = "Heading Error";
        kv6.value = std::to_string(data_.heading_error);

        diagnostic_msg.values = {kv1, kv2, kv3, kv4, kv5, kv6};

        // Determine the level and message based on error checks
        if (position_error)
        {
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_msg.message = "Position error detected by status flags";
        }
        else if (orientation_error)
        {
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_msg.message = "Orientation error detected by status flags";
        }
        else
        {
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diagnostic_msg.message = "All systems nominal";
        }

        diagnostic_array.status.push_back(diagnostic_msg);
        return diagnostic_array;
    }

    nav_msgs::msg::Odometry Driver::get_odometry_message() const
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = time_;

        odom_msg.header.frame_id = "seapath";
        odom_msg.child_frame_id = "seapath";
       
        auto [x, y, z] = lla2flat(data_.latitude, data_.longitude, data_.ellipsoid_height);


        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(deg2rad(data_.roll), deg2rad(data_.pitch), deg2rad(data_.heading));

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.pose.covariance[0] = data_.longitude_error * data_.longitude_error;
        odom_msg.pose.covariance[7] = data_.latitude_error * data_.latitude_error;
        odom_msg.pose.covariance[14] = data_.height_error * data_.height_error;
        odom_msg.pose.covariance[21] = data_.roll_error * data_.roll_error;
        odom_msg.pose.covariance[28] = data_.pitch_error * data_.pitch_error;
        odom_msg.pose.covariance[35] = data_.heading_error * data_.heading_error;

        odom_msg.twist.twist.linear.x = data_.north_velocity;
        odom_msg.twist.twist.linear.y = data_.east_velocity;
        odom_msg.twist.twist.linear.z = data_.down_velocity;

        odom_msg.twist.twist.angular.x = data_.roll_rate;
        odom_msg.twist.twist.angular.y = data_.pitch_rate;
        odom_msg.twist.twist.angular.z = data_.yaw_rate;

        odom_msg.twist.covariance[0] = data_.latitude_error * data_.latitude_error;
        odom_msg.twist.covariance[7] = data_.longitude_error * data_.longitude_error;
        odom_msg.twist.covariance[14] = data_.height_error * data_.height_error;

        // Temp hack to avoid inf in covariance
        if (data_.heading_error > 10.0)
        {
            odom_msg.twist.covariance[35] = 10.0;
        }
        else
        {
            odom_msg.twist.covariance[35] = data_.heading_error * data_.heading_error;
        }
        odom_msg.twist.covariance[21] = data_.roll_error * data_.roll_error;
        odom_msg.twist.covariance[28] = data_.pitch_error * data_.pitch_error;

        return odom_msg;
    }

    
    sensor_msgs::msg::NavSatFix Driver::get_navsatfix_message() const
    {
        sensor_msgs::msg::NavSatFix nav_msg;
      
        nav_msg.header.stamp = time_;
        nav_msg.header.frame_id = "seapath"; // ??

        nav_msg.latitude = data_.latitude;
        nav_msg.longitude = data_.longitude;
        nav_msg.altitude = data_.ellipsoid_height;

        // Set the position covariance based on the errors provided
        nav_msg.position_covariance[0] = data_.longitude_error * data_.longitude_error;
        nav_msg.position_covariance[4] = data_.latitude_error * data_.latitude_error;  
        nav_msg.position_covariance[8] = data_.height_error * data_.height_error;   

        // Set the remaining elements of the covariance matrix to zero
        nav_msg.position_covariance[1] = 0.0;
        nav_msg.position_covariance[2] = 0.0;
        nav_msg.position_covariance[3] = 0.0;
        nav_msg.position_covariance[5] = 0.0;
        nav_msg.position_covariance[6] = 0.0;
        nav_msg.position_covariance[7] = 0.0;

        // Specify the type of covariance
        nav_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        return nav_msg;
    }

    vortex_msgs::msg::KMBinary Driver::get_kmbinary_message() const
    {
        vortex_msgs::msg::KMBinary kmb_msg;

        kmb_msg.utc_seconds = data_.utc_seconds;
        kmb_msg.utc_nanoseconds = data_.utc_nanoseconds;

        kmb_msg.status = data_.status;

        kmb_msg.latitude = data_.latitude;
        kmb_msg.longitude = data_.longitude;
        kmb_msg.ellipsoid_height = data_.ellipsoid_height;

        kmb_msg.roll = data_.roll;
        kmb_msg.pitch = data_.pitch;
        kmb_msg.heading = data_.heading;
        kmb_msg.heave = data_.heave;

        kmb_msg.roll_rate = data_.roll_rate;
        kmb_msg.pitch_rate = data_.pitch_rate;
        kmb_msg.yaw_rate = data_.yaw_rate;

        kmb_msg.north_velocity = data_.north_velocity;
        kmb_msg.east_velocity = data_.east_velocity;
        kmb_msg.down_velocity = data_.down_velocity;

        kmb_msg.latitude_error = data_.latitude_error;
        kmb_msg.longitude_error = data_.longitude_error;
        kmb_msg.height_error = data_.height_error;
        kmb_msg.roll_error = data_.roll_error;
        kmb_msg.pitch_error = data_.pitch_error;
        kmb_msg.heading_error = data_.heading_error;
        kmb_msg.heave_error = data_.heave_error;

        kmb_msg.north_acceleration = data_.north_acceleration;
        kmb_msg.east_acceleration = data_.east_acceleration;
        kmb_msg.down_acceleration = data_.down_acceleration;

        kmb_msg.delayed_heave_utc_seconds = data_.delayed_heave_utc_seconds;
        kmb_msg.delayed_heave_utc_nanoseconds = data_.delayed_heave_utc_nanoseconds;
        kmb_msg.delayed_heave = data_.delayed_heave;

        return kmb_msg;
    }

    double Driver::deg2rad(const double& degrees) const
    {
        return degrees * (M_PI / 180.0);
    }

    std::array<double, 3> Driver::lla2flat(const double& lat, const double& lon, const double& alt) const
    {

        const double R = 6378137.0; // WGS-84 Earth semimajor axis (meters)
        const double f = 1.0 / 298.257223563; // Flattening of the earth
        double psi_rad = 0.0; // Angular direction of the flat Earth x-axis, specified as a scalar. 
            // The angular direction is the degrees clockwise from north, 
            // which is the angle in degrees used for converting flat Earth x and y coordinates to the north and east coordinates

        // Convert angles from degrees to radians
        double lat_rad = deg2rad(lat);
        double lon_rad = deg2rad(lon);
        double origin_lat_rad = deg2rad(origin_lat_);
        double origin_lon_rad = deg2rad(origin_lon_);

        // Calculate delta latitude and delta longitude in radians
        double dlat = lat_rad - origin_lat_rad;
        double dlon = lon_rad - origin_lon_rad;

        // Radius of curvature in the vertical prime (RN)
        double RN = R / sqrt(1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));
        
        // Radius of curvature in the meridian (RM)
        double RM = RN * (1.0 - (2.0 * f - f * f)) / (1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

        // Changes in the north (dN) and east (dE) positions
        double dN = RM * dlat;
        double dE = RN * cos(origin_lat_rad) * dlon;

        // Transformation from North-East to flat x-y coordinates
        double px = cos(psi_rad) * dN - sin(psi_rad) * dE;
        double py = sin(psi_rad) * dN + cos(psi_rad) * dE;
        double pz = -alt - origin_h_; // Flat Earth z-axis value (downwards positive, NED)

        return {px, py, pz};
    }


    void Driver::publish_dyn_tf() const 
    {
        auto [x, y, z] = lla2flat(data_.latitude, data_.longitude, data_.ellipsoid_height);
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(x, y, z));

        transform.setRotation(tf2::Quaternion(1, 0, 0, 0));

        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.transform = tf2::toMsg(transform);
        transform_msg.header.stamp = time_;
        transform_msg.header.frame_id = "world";
        transform_msg.child_frame_id = "vechicle";
        tf_broadcaster_->sendTransform(transform_msg);

        tf2::Quaternion q;
        q.setRPY(0, 0, deg2rad(data_.heading));
        transform.setRotation(q);
        transform_msg.transform = tf2::toMsg(transform);
        transform_msg.child_frame_id = "vehicle-1";
        tf_broadcaster_->sendTransform(transform_msg);

        q.setRPY(0, deg2rad(data_.pitch), deg2rad(data_.heading));
        transform.setRotation(q);
        transform_msg.transform = tf2::toMsg(transform);
        transform_msg.child_frame_id = "vehicle-2";
        tf_broadcaster_->sendTransform(transform_msg);

        q.setRPY(deg2rad(data_.roll), deg2rad(data_.pitch), deg2rad(data_.heading));
        transform.setRotation(q);
        transform_msg.transform = tf2::toMsg(transform);
        transform_msg.child_frame_id = "seapath";
        tf_broadcaster_->sendTransform(transform_msg);

    }

    std::ostream &operator<<(std::ostream &os, const KMBinaryData &data)
    {
        os << "Start ID: " << data.start_id << std::endl;
        os << "Dgm Length: " << data.dgm_length << std::endl;
        os << "Dgm Version: " << data.dgm_version << std::endl;
        os << "UTC Seconds: " << data.utc_seconds << std::endl;
        os << "UTC Nanoseconds: " << data.utc_nanoseconds << std::endl;
        os << "Status: " << data.status << std::endl;
        os << "Latitude: " << data.latitude << std::endl;
        os << "Longitude: " << data.longitude << std::endl;
        os << "Ellipsoid Height: " << data.ellipsoid_height << std::endl;
        os << "Roll: " << data.roll << std::endl;
        os << "Pitch: " << data.pitch << std::endl;
        os << "Heading: " << data.heading << std::endl;
        os << "Heave: " << data.heave << std::endl;
        os << "Roll Rate: " << data.roll_rate << std::endl;
        os << "Pitch Rate: " << data.pitch_rate << std::endl;
        os << "Yaw Rate: " << data.yaw_rate << std::endl;
        os << "North Velocity: " << data.north_velocity << std::endl;
        os << "East Velocity: " << data.east_velocity << std::endl;
        os << "Down Velocity: " << data.down_velocity << std::endl;
        os << "Latitude Error: " << data.latitude_error << std::endl;
        os << "Longitude Error: " << data.longitude_error << std::endl;
        os << "Height Error: " << data.height_error << std::endl;
        os << "Roll Error: " << data.roll_error << std::endl;
        os << "Pitch Error: " << data.pitch_error << std::endl;
        os << "Heading Error: " << data.heading_error << std::endl;
        os << "Heave Error: " << data.heave_error << std::endl;
        os << "North Acceleration: " << data.north_acceleration << std::endl;
        os << "East Acceleration: " << data.east_acceleration << std::endl;
        os << "Down Acceleration: " << data.down_acceleration << std::endl;
        os << "Delayed Heave UTC Seconds: " << data.delayed_heave_utc_seconds << std::endl;
        os << "Delayed Heave UTC Nanoseconds: " << data.delayed_heave_utc_nanoseconds << std::endl;
        os << "Delayed Heave: " << data.delayed_heave << std::endl;
        return os;
    }

    void printKMBinaryData(const KMBinaryData &data)
    {
        std::cout << data;
    }
} // namespace seapath