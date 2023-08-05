#include "seapath_gnss_ros_driver/seapath_ros_driver.hpp"

SeaPathRosDriver::SeaPathRosDriver(ros::NodeHandle nh, const char* UDP_IP, const int UDP_PORT) : nh{nh}, seaPathSocket(UDP_IP, UDP_PORT) {
    nav_pub = nh.advertise<sensor_msgs::NavSatFix>("/sensor/gnss", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/sensor/gnss/odom", 10);

}

SeapathData SeaPathRosDriver::getSeapathData() {
    std::string nmea_data = seaPathSocket.receiveData();
    return parseNmeaData(nmea_data);
}

SeapathData SeaPathRosDriver::parseNmeaData(std::string nmea_data) {
    SeapathData seapath_data;
    std::istringstream dataStream(nmea_data);
    std::string sentence;

    while (getline(dataStream, sentence, '\n')) {
        std::string data = sentence.substr(1, sentence.find('*') - 1);
        std::vector<std::string> parts;
        std::string part;
        std::istringstream partStream(data);
        while (getline(partStream, part, ',')) {
            parts.push_back(part);
        }

        if (parts[0] == "INGGA") { // GNSS fix

            if(parts[2].empty() || parts[4].empty() || parts[9].empty()) {
                continue;
            }

            std::string north = parts[2];
            std::string east = parts[4];
            std::string altitude = parts[9];


            seapath_data.north = convert_dms_to_dd(std::stod(north));
            seapath_data.east = convert_dms_to_dd(std::stod(east));
            seapath_data.altitude = std::stod(altitude);

            if (ORIGIN_N == 0.0) {
                ORIGIN_N = seapath_data.north;
                ORIGIN_E = seapath_data.east;
            }

            auto [n, e] = displacement_wgs84(seapath_data.north, seapath_data.east);
            seapath_data.x_displacement = n;
            seapath_data.y_displacement = e;
            //std::cout << "N [m]: " << n << " \t E [m]: " << e << std::endl;
        }

        if (parts[0] == "INHDT") { // True heading
            if(parts[1].empty()) {
                continue;
            }

            std::string true_north_heading = parts[1];

            seapath_data.heading = std::stod(true_north_heading);
        }

        if (parts[0] == "INVTG") { // Course over ground and speed over ground
            if(parts[1].empty() || parts[7].empty()) {
                continue;
            }
            std::string ground_speed_mps = parts[7];
            std::string ground_course_rad = parts[1];

            seapath_data.sog_mps = std::round(std::stod(ground_speed_mps) * 1000.0 / 3600.0 * 1e6) / 1e6;
            seapath_data.cog_rad = std::round(std::stod(ground_course_rad) * 2 * M_PI / 360.0 * 1e6) / 1e6;
        }

        if (parts[0] == "GNGST") { // GNSS pseudorange error statistics
            if (parts[6].empty() || parts[7].empty() || parts[8].empty()) {
                continue;
            }

            std::string sigma_lat = parts[6];
            std::string sigma_lon = parts[7];
            std::string sigma_alt = parts[8];

            seapath_data.north_sigma = std::stod(sigma_lat);
            seapath_data.east_sigma = std::stod(sigma_lon);
            seapath_data.altitude_sigma = std::stod(sigma_alt);
        }

        if (parts[0] == "INROT") { // Rate of turn
            if(parts[1].empty()) {
                continue;
            }
            std::string rate_of_turn = parts[1];

            seapath_data.rot = std::stod(rate_of_turn);
        }
    }

    return seapath_data;
}

void SeaPathRosDriver::publish(SeapathData data) {
    sensor_msgs::NavSatFix msg;
    double epsilon = 1e-9;
    if (abs(data.north) < epsilon || abs(data.east) < epsilon) {
        ROS_WARN("Invalid Seapath GNSS data received, skipping...\nData values: north=%f, east=%f", data.north, data.east);
        return; // do not publish zero msgs
    }

    msg.header.frame_id = "gnss";
    msg.header.stamp = ros::Time::now();

    msg.latitude = data.north;
    msg.longitude = data.east;
    msg.altitude = data.altitude;

    msg.position_covariance[0] = data.north_sigma * data.north_sigma;
    msg.position_covariance[4] = data.east_sigma * data.east_sigma;
    msg.position_covariance[8] = data.altitude_sigma * data.altitude_sigma;

    msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    nav_pub.publish(msg);

    // ENU!!!!
    double sigma_floor = 1e-7;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.frame_id = "gnss";
    odometry_msg.header.stamp = ros::Time::now();

    odometry_msg.pose.pose.position.x = data.y_displacement;
    odometry_msg.pose.pose.position.y = data.x_displacement;

    odometry_msg.pose.covariance[0] = sigma_floor; //data.north_sigma * data.north_sigma + sigma_floor; // X variance
    odometry_msg.pose.covariance[7] = sigma_floor; //data.east_sigma * data.east_sigma + sigma_floor;   // Y variance

    odom_pub.publish(odometry_msg);
}

std::pair<double, double> SeaPathRosDriver::displacement_wgs84(double north, double east) {
    double R = 6371.0;
    double m_per_deg_lat = R * 1000 * (M_PI / 180.0);
    double m_per_deg_lon = m_per_deg_lat * cos(ORIGIN_N * M_PI / 180.0);

    double displacement_north = (north - ORIGIN_N) * m_per_deg_lat;
    double displacement_east = (east - ORIGIN_E) * m_per_deg_lon;

    return {displacement_north, displacement_east};
}

double SeaPathRosDriver::convert_dms_to_dd(double dms) {
    double degrees = static_cast<int>(dms / 100);
    double minutes = dms - (degrees * 100);
    double dd = degrees + (minutes / 60);
    return round(dd * 1e6) / 1e6;
}
