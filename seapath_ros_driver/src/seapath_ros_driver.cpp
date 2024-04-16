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
    
        std::thread(&Driver::setup_socket, this).detach();
        initial_setup();

    }

    void Driver::setup_socket()
    {
        socket_.set_ip(UDP_IP_);
        socket_.set_port(UDP_PORT_);
        socket_.create_socket();
        socket_.bind_socket();
        socket_.receive_data();
    }

    void Driver::reset_origin()
    {
         // Block until a valid data message is received
        while (!origin_set_) {
            if (socket_.get_data_status()) {
                auto data = socket_.get_kmbinary_data();
                auto time = this->get_clock()->now();
                set_origin(data);
                origin_pub_->publish(get_navsatfix_message(data, time));
                publish_static_tf(time);
                kmbinary_pub_->publish(get_kmbinary_message(data));
                diagnostic_pub_->publish(get_diagnostic_array(data, time));
                nav_pub_->publish(get_navsatfix_message(data, time));
                odom_pub_->publish(get_odometry_message(data, time));
                publish_dyn_tf(data, time);  
                        

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
           auto data = socket_.get_kmbinary_data();
           auto time = this->get_clock()->now();

            kmbinary_pub_->publish(get_kmbinary_message(data));
            diagnostic_pub_->publish(get_diagnostic_array(data, time));
            if(!data.status_ok()){
                return;
            }
            nav_pub_->publish(get_navsatfix_message(data, time));
            odom_pub_->publish(get_odometry_message(data, time));
            publish_dyn_tf(data, time); 

        
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

    void Driver::publish_static_tf(const rclcpp::Time& time) const
        {
        // Setup the transform
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = time;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "world_foxglove";

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

    void Driver::set_origin(const KMBinaryData& data)
    {
        origin_lat_ = data.latitude;
        origin_lon_ = data.longitude;
        origin_h_ = data.ellipsoid_height;
        origin_set_ = true;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Setting global origin to: " << origin_lat_ << ", " << origin_lon_ << ", " << origin_h_ << "\n");
    }

    

    diagnostic_msgs::msg::DiagnosticArray Driver::get_diagnostic_array(const KMBinaryData& data, const rclcpp::Time& time) const
    {
        diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_msg;
        diagnostic_msg.name = "Diagnostic_seapath_status";
        diagnostic_array.header.stamp = time;

        // Check socket connection first
        if (!socket_.socket_connected())
        {
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diagnostic_msg.message = "Socket disconnected";
            diagnostic_array.status.push_back(diagnostic_msg);
            return diagnostic_array;
        }

        // Create KeyValue pairs for the diagnostic message
        diagnostic_msgs::msg::KeyValue kv1, kv2, kv3, kv4, kv5, kv6;
        diagnostic_msgs::msg::KeyValue kv7, kv8, kv9, kv10, kv11, kv12;

        kv1.key = "Latitude Error";
        kv1.value = std::to_string(data.latitude_error);
        
        kv2.key = "Longitude Error";
        kv2.value = std::to_string(data.longitude_error);

        kv3.key = "Height Error";
        kv3.value = std::to_string(data.height_error);

        kv4.key = "Roll Error";
        kv4.value = std::to_string(data.roll_error);

        kv5.key = "Pitch Error";
        kv5.value = std::to_string(data.pitch_error);

        kv6.key = "Heading Error";
        kv6.value = std::to_string(data.heading_error);

        auto flags = data.evaluateDiagnosticStatus();


        kv7.key = "Horizontal position and velocity status";
        kv7.value = KMBinaryData::status_to_string(flags.horizontal_pos_vel);

        kv8.key = "Roll and pitch status";
        kv8.value = KMBinaryData::status_to_string(flags.roll_pitch);
        
        kv9.key = "Heading status";
        kv9.value = KMBinaryData::status_to_string(flags.heading);

        kv10.key = "Heave and vertical velocity status";
        kv10.value = KMBinaryData::status_to_string(flags.heave_vert_vel);

        kv11.key = "Acceleration status";
        kv11.value = KMBinaryData::status_to_string(flags.acceleration);

        kv12.key = "Delayed heave status";
        kv12.value = KMBinaryData::status_to_string(flags.delayed_heave);

        diagnostic_msg.values = {kv1, kv2, kv3, kv4, kv5, kv6, kv7, kv8, kv9, kv10, kv11, kv12};

        // Determine the level and message based on error checks
        switch (data.determineOverallStatus())
        {
        case KMBinaryData::DiagnosticStatus::INVALID:
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diagnostic_msg.message = "Invalid data";
            break;
        case KMBinaryData::DiagnosticStatus::REDUCED:
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_msg.message = "Reduced performance";
            break;
        case KMBinaryData::DiagnosticStatus::OK:
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diagnostic_msg.message = "All systems nominal";
            break;
        }
        
        diagnostic_array.status.push_back(diagnostic_msg);
        return diagnostic_array;
    }

    nav_msgs::msg::Odometry Driver::get_odometry_message(const KMBinaryData& data, const rclcpp::Time& time) const
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = time;

        odom_msg.header.frame_id = "seapath";
        odom_msg.child_frame_id = "seapath";
       
        auto [x, y, z] = lla2flat(data.latitude, data.longitude, data.ellipsoid_height);


        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(deg2rad(data.roll), deg2rad(data.pitch), deg2rad(data.heading));

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.pose.covariance[0] = data.longitude_error * data.longitude_error;
        odom_msg.pose.covariance[7] = data.latitude_error * data.latitude_error;
        odom_msg.pose.covariance[14] = data.height_error * data.height_error;
        odom_msg.pose.covariance[21] = data.roll_error * data.roll_error;
        odom_msg.pose.covariance[28] = data.pitch_error * data.pitch_error;
        odom_msg.pose.covariance[35] = data.heading_error * data.heading_error;

        odom_msg.twist.twist.linear.x = data.north_velocity;
        odom_msg.twist.twist.linear.y = data.east_velocity;
        odom_msg.twist.twist.linear.z = data.down_velocity;

        odom_msg.twist.twist.angular.x = data.roll_rate;
        odom_msg.twist.twist.angular.y = data.pitch_rate;
        odom_msg.twist.twist.angular.z = data.yaw_rate;

        odom_msg.twist.covariance[0] = data.latitude_error * data.latitude_error;
        odom_msg.twist.covariance[7] = data.longitude_error * data.longitude_error;
        odom_msg.twist.covariance[14] = data.height_error * data.height_error;

        // Temp hack to avoid inf in covariance
        if (data.heading_error > 10.0)
        {
            odom_msg.twist.covariance[35] = 10.0;
        }
        else
        {
            odom_msg.twist.covariance[35] = data.heading_error * data.heading_error;
        }
        odom_msg.twist.covariance[21] = data.roll_error * data.roll_error;
        odom_msg.twist.covariance[28] = data.pitch_error * data.pitch_error;

        return odom_msg;
    }

    
    sensor_msgs::msg::NavSatFix Driver::get_navsatfix_message(const KMBinaryData& data, const rclcpp::Time& time) const
    {
        sensor_msgs::msg::NavSatFix nav_msg;
      
        nav_msg.header.stamp = time;
        nav_msg.header.frame_id = "lla";

        nav_msg.latitude = data.latitude;
        nav_msg.longitude = data.longitude;
        nav_msg.altitude = data.ellipsoid_height;

        // Set the position covariance based on the errors provided
        nav_msg.position_covariance[0] = data.longitude_error * data.longitude_error;
        nav_msg.position_covariance[4] = data.latitude_error * data.latitude_error;  
        nav_msg.position_covariance[8] = data.height_error * data.height_error;   

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

    vortex_msgs::msg::KMBinary Driver::get_kmbinary_message(const KMBinaryData& data) const
    {
        vortex_msgs::msg::KMBinary kmb_msg;

        kmb_msg.utc_seconds = data.utc_seconds;
        kmb_msg.utc_nanoseconds = data.utc_nanoseconds;

        kmb_msg.status = data.status;

        kmb_msg.latitude = data.latitude;
        kmb_msg.longitude = data.longitude;
        kmb_msg.ellipsoid_height = data.ellipsoid_height;

        kmb_msg.roll = data.roll;
        kmb_msg.pitch = data.pitch;
        kmb_msg.heading = data.heading;
        kmb_msg.heave = data.heave;

        kmb_msg.roll_rate = data.roll_rate;
        kmb_msg.pitch_rate = data.pitch_rate;
        kmb_msg.yaw_rate = data.yaw_rate;

        kmb_msg.north_velocity = data.north_velocity;
        kmb_msg.east_velocity = data.east_velocity;
        kmb_msg.down_velocity = data.down_velocity;

        kmb_msg.latitude_error = data.latitude_error;
        kmb_msg.longitude_error = data.longitude_error;
        kmb_msg.height_error = data.height_error;
        kmb_msg.roll_error = data.roll_error;
        kmb_msg.pitch_error = data.pitch_error;
        kmb_msg.heading_error = data.heading_error;
        kmb_msg.heave_error = data.heave_error;

        kmb_msg.north_acceleration = data.north_acceleration;
        kmb_msg.east_acceleration = data.east_acceleration;
        kmb_msg.down_acceleration = data.down_acceleration;

        kmb_msg.delayed_heave_utc_seconds = data.delayed_heave_utc_seconds;
        kmb_msg.delayed_heave_utc_nanoseconds = data.delayed_heave_utc_nanoseconds;
        kmb_msg.delayed_heave = data.delayed_heave;

        return kmb_msg;
    }

    constexpr double Driver::deg2rad(double degrees) const
    {
        return degrees * (M_PI / 180.0);
    }

    std::array<double, 3> Driver::lla2flat(double lat, double lon, double alt) const
    {

        const double R = 6378137.0; // WGS-84 Earth semimajor axis (meters)
        const double f = 1.0 / 298.257223563; // Flattening of the earth
        const double psi_rad = 0.0; // Angular direction of the flat Earth x-axis, specified as a scalar. 
            // The angular direction is the degrees clockwise from north, 
            // which is the angle in degrees used for converting flat Earth x and y coordinates to the north and east coordinates

        // Convert angles from degrees to radians
        const double lat_rad = deg2rad(lat);
        const double lon_rad = deg2rad(lon);
        const double origin_lat_rad = deg2rad(origin_lat_);
        const double origin_lon_rad = deg2rad(origin_lon_);

        // Calculate delta latitude and delta longitude in radians
        const double dlat = lat_rad - origin_lat_rad;
        const double dlon = lon_rad - origin_lon_rad;

        // Radius of curvature in the vertical prime (RN)
        const double RN = R / sqrt(1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));
        
        // Radius of curvature in the meridian (RM)
        const double RM = RN * (1.0 - (2.0 * f - f * f)) / (1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

        // Changes in the north (dN) and east (dE) positions
        const double dN = RM * dlat;
        const double dE = RN * cos(origin_lat_rad) * dlon;

        // Transformation from North-East to flat x-y coordinates
        const double px = cos(psi_rad) * dN - sin(psi_rad) * dE;
        const double py = sin(psi_rad) * dN + cos(psi_rad) * dE;
        const double pz = origin_h_ - alt; // Flat Earth z-axis value (downwards positive, NED)

        return {px, py, pz};
    }


    void Driver::publish_dyn_tf(const KMBinaryData& data, const rclcpp::Time& time) const 
    {
        auto [x, y, z] = lla2flat(data.latitude, data.longitude, data.ellipsoid_height);
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(x, y, z));

        transform.setRotation(tf2::Quaternion(0, 0, 0, 1));

        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.transform = tf2::toMsg(transform);
        transform_msg.header.stamp = time;
        transform_msg.header.frame_id = "world";
        transform_msg.child_frame_id = "vehicle";
        tf_broadcaster_->sendTransform(transform_msg);

        tf2::Quaternion q;
        q.setRPY(0, 0, deg2rad(data.heading));
        transform.setRotation(q);
        transform_msg.transform = tf2::toMsg(transform);
        transform_msg.child_frame_id = "vehicle-1";
        tf_broadcaster_->sendTransform(transform_msg);

        q.setRPY(0, deg2rad(data.pitch), deg2rad(data.heading));
        transform.setRotation(q);
        transform_msg.transform = tf2::toMsg(transform);
        transform_msg.child_frame_id = "vehicle-2";
        tf_broadcaster_->sendTransform(transform_msg);

        q.setRPY(deg2rad(data.roll), deg2rad(data.pitch), deg2rad(data.heading));
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