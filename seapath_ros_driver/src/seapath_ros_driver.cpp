#include "seapath_ros_driver.hpp"

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

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sensor/seapath/odom/ned", qos_sensor_data);
        origin_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/sensor/seapath/origin", qos_transient_local);
        diagnosticArray_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/sensor/seapath/diagnostic_array", qos_transient_local);
        nav_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/sensor/seapath/NavSatFix", qos_sensor_data);
        kmbinary_pub_ = this->create_publisher<vortex_msgs::msg::KMBinary>("/sensor/seapath/vortex_msgs", qos_sensor_data);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        declare_parameter<std::string>("UDP_IP", "10.0.1.10");
        declare_parameter<u_int16_t>("UDP_PORT", 31421);
        UDP_IP_ = get_parameter("UDP_IP").as_string();
        UDP_PORT_ = get_parameter("UDP_PORT").as_int();
        shared_vector_ = std::vector<uint8_t>();
        packet_ready_ = false;
        socket_connected_ = false;
        
        std::thread(&Driver::SetupSocket, this, UDP_IP_, UDP_PORT_).detach();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Driver::timer_callback, this));
    }

    void Driver::SetupSocket(std::string UDP_IP_, uint16_t UDP_PORT_)
    {
        Socket Socket(UDP_IP_, UDP_PORT_, shared_vector_, mutex_, packet_ready_, socket_connected_);
        Socket.create_socket();
        Socket.connect_to_socket();
        Socket.receive_data();
    }

    void Driver::timer_callback()
    {
        try
        {
            if (!packet_ready_)
            {
                return;
            }

            std::unique_lock<std::mutex> lock(mutex_); // Thread-safe handling
            process_kmbinary_data(shared_vector_);
            publish(data_);

            packet_ready_ = false;
            lock.unlock();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void Driver::publish(KMBinaryData data)
    {

        auto odom = get_odometry_message(data);
        auto current_diagnostic = get_diagnostic_message();
        auto diagnostic_array = get_diagnostic_array(current_diagnostic);
        auto navSatFix = get_navsatfix_message(data);
        auto KMBinaryData = get_kmbinary_message(data);
        auto transform = get_transform_message(data);
        diagnosticArray_pub_->publish(diagnostic_array);

        if (current_diagnostic.level == diagnostic_msgs::msg::DiagnosticStatus::OK)
        {
            odom_pub_->publish(odom);
            tf_broadcaster_->sendTransform(transform);
            nav_pub_->publish(navSatFix);
            kmbinary_pub_->publish(KMBinaryData);
        }
    }

    void Driver::process_kmbinary_data(std::vector<uint8_t> data)
    {
        data_ = parse_kmbinary_data(data);
    }

    KMBinaryData Driver::parse_kmbinary_data(std::vector<uint8_t> data)
    {
        KMBinaryData result;
        size_t offset = 0;

        /**
         * @brief Helper lambda to copy data and update the offset
         *
         */
        auto copyData = [&data, &offset](void *dest, size_t size)
        {
            std::memcpy(dest, data.data() + offset, size);
            offset += size;
        };
        if (socket_connected_)
        {
            copyData(result.start_id, 4);
            copyData(&result.dgm_length, 2);
            copyData(&result.dgm_version, 2);
            copyData(&result.utc_seconds, 4);
            copyData(&result.utc_nanoseconds, 4);
            copyData(&result.status, 4);
            copyData(&result.latitude, 8);
            copyData(&result.longitude, 8);
            copyData(&result.ellipsoid_height, 4);
            copyData(&result.roll, 4);
            copyData(&result.pitch, 4);
            copyData(&result.heading, 4);
            copyData(&result.heave, 4);
            copyData(&result.roll_rate, 4);
            copyData(&result.pitch_rate, 4);
            copyData(&result.yaw_rate, 4);
            copyData(&result.north_velocity, 4);
            copyData(&result.east_velocity, 4);
            copyData(&result.down_velocity, 4);
            copyData(&result.latitude_error, 4);
            copyData(&result.longitude_error, 4);
            copyData(&result.height_error, 4);
            copyData(&result.roll_error, 4);
            copyData(&result.pitch_error, 4);
            copyData(&result.heading_error, 4);
            copyData(&result.heave_error, 4);
            copyData(&result.north_acceleration, 4);
            copyData(&result.east_acceleration, 4);
            copyData(&result.down_acceleration, 4);
            copyData(&result.delayed_heave_utc_seconds, 4);
            copyData(&result.delayed_heave_utc_nanoseconds, 4);
            copyData(&result.delayed_heave, 4);
        }

        return result;
    }

    diagnostic_msgs::msg::DiagnosticStatus Driver::get_diagnostic_message()
    {
        // check if it's properly connected to the socket
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_msg;
        if (socket_connected_ == false)
        {
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_msg.name = "Diagnostic_connection_to_socket_status";
            diagnostic_msg.message = "Socket disconnected";
        }
        else if (socket_connected_ == true)
        {
            diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diagnostic_msg.name = "Diagnostic_connection_to_socket_status";
            diagnostic_msg.message = "Socket connection is OK";
        }
        return diagnostic_msg;
    }

    diagnostic_msgs::msg::DiagnosticArray Driver::get_diagnostic_array(diagnostic_msgs::msg::DiagnosticStatus diagnostic_msg)
    {
        diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
        diagnostic_array.status.push_back(diagnostic_msg);
        return diagnostic_array;
    }

    nav_msgs::msg::Odometry Driver::get_odometry_message(const KMBinaryData &data)
    {
        nav_msgs::msg::Odometry odom_msg;
        rclcpp::Time current_time;
        odom_msg.header.stamp = current_time = this->now();

        odom_msg.header.frame_id = "seapath_frame_pose";
        odom_msg.child_frame_id = "seapath_frame_twist";
        float north = data.latitude;
        float east = data.longitude;
        float height = data.ellipsoid_height;

        if (ORIGIN_N == -100 && ORIGIN_E == -100 && ORIGIN_H == -100)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Setting global starting origin to: " << north << ", " << east << "\n");
            ORIGIN_N = north;
            ORIGIN_E = east;
            ORIGIN_H = height;
        }

        auto xy = displacement_wgs84(north, east);

        odom_msg.pose.pose.position.x = xy.first;
        odom_msg.pose.pose.position.y = xy.second;
        odom_msg.pose.pose.position.z = height - ORIGIN_H;

        tf2::Quaternion q;
        q.setRPY(data.roll, data.pitch, data.heading);

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

    geometry_msgs::msg::Point Driver::get_origin_message()
    {

        geometry_msgs::msg::Point origin_msg;
        origin_msg.set__x(ORIGIN_N);
        origin_msg.set__y(ORIGIN_E);
        origin_msg.set__z(ORIGIN_H);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New Origin: "
                                                             << "N:" << origin_msg.x << ", E:" << origin_msg.y << "H: " << origin_msg.z << "\n");
        return origin_msg;
    }

    sensor_msgs::msg::NavSatFix Driver::get_navsatfix_message(const KMBinaryData &data)
    {
        sensor_msgs::msg::NavSatFix nav_msg;
        rclcpp::Time current_time;
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
        pose_cov.header.stamp = current_time = this->now();
        pose_cov.header.frame_id = "world_frame";
        pose_cov.pose = get_odometry_message(data).pose;

        nav_msg.header.stamp = current_time = this->now();
        nav_msg.header.frame_id = "world_frame";

        nav_msg.latitude = data.latitude;
        nav_msg.longitude = data.longitude;
        nav_msg.altitude = data.ellipsoid_height;

        for (int i = 0; i < 3; ++i)
        {
            nav_msg.position_covariance[i + 0] = pose_cov.pose.covariance[i + 0];
            nav_msg.position_covariance[i + 3] = pose_cov.pose.covariance[i + 7];
            nav_msg.position_covariance[i + 6] = pose_cov.pose.covariance[i + 14];
        }

        return nav_msg;
    }

    vortex_msgs::msg::KMBinary Driver::get_kmbinary_message(const KMBinaryData &data)
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

    std::pair<double, double> Driver::displacement_wgs84(double north, double east)
    {
        double R = 6371.0;
        double m_per_deg_lat = R * 1000 * (M_PI / 180.0);
        double m_per_deg_lon = m_per_deg_lat * cos(ORIGIN_N * M_PI / 180.0);

        double displacement_north = (north - ORIGIN_N) * m_per_deg_lat;
        double displacement_east = (east - ORIGIN_E) * m_per_deg_lon;

        return {displacement_north, displacement_east};
    }

    void Driver::reset_origin(const KMBinaryData &data)
    {

        float north = data.latitude;
        float east = data.longitude;
        float height = data.ellipsoid_height;

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Setting global origin to: " << north << ", " << east << "\n");
        ORIGIN_N = north;
        ORIGIN_E = east;
        ORIGIN_H = height;
    }

    double Driver::convert_dms_to_dd(double dms)
    {
        double degrees = static_cast<int>(dms / 100);
        double minutes = dms - (degrees * 100);
        double dd = degrees + (minutes / 60);
        return round(dd * 1e6) / 1e6;
    }

    geometry_msgs::msg::TransformStamped Driver::get_transform_message(const KMBinaryData &data)
    {
        geometry_msgs::msg::TransformStamped transform_msg;
        rclcpp::Time current_time;
        transform_msg.header.stamp = current_time = this->now();
        transform_msg.header.frame_id = "world_frame";
        transform_msg.child_frame_id = "seapath_frame_pose";

        float north = data.latitude;
        float east = data.longitude;
        float height = data.ellipsoid_height;

        auto xy = displacement_wgs84(north, east);

        transform_msg.transform.translation.x = xy.first;
        transform_msg.transform.translation.y = xy.second;
        transform_msg.transform.translation.z = height - ORIGIN_H;

        tf2::Quaternion q;
        q.setRPY(data.roll, data.pitch, data.heading);

        transform_msg.transform.rotation.x = q.x();
        transform_msg.transform.rotation.y = q.y();
        transform_msg.transform.rotation.z = q.z();
        transform_msg.transform.rotation.w = q.w();

        return transform_msg;
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