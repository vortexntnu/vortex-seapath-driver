#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <ctime>
#include <sstream>
#include <algorithm>

const double pi = 3.14159265358979323846;

double ORIGIN_N = 0.0;
double ORIGIN_E = 0.0;

struct SeapathData {
    double north = 0.0;
    double north_sigma = 0.0;

    double east = 0.0;
    double east_sigma = 0.0;

    double altitude = 0.0;
    double altitude_sigma = 0.0;

    double heading = 0.0;

    double sog_mps = 0.0;
    double cog_rad = 0.0;

    double rot = 0.0;
};

std::pair<double, double> displacement_wgs84(double north, double east) {
    double R = 6371.0;
    double m_per_deg_lat = R * 1000 * (pi / 180.0);
    double m_per_deg_lon = m_per_deg_lat * cos(ORIGIN_N * pi / 180.0);

    double displacement_north = (north - ORIGIN_N) * m_per_deg_lat;
    double displacement_east = (east - ORIGIN_E) * m_per_deg_lon;

    return {displacement_north, displacement_east};
}

double convert_dms_to_dd(double dms) {
    double degrees = static_cast<int>(dms / 100);
    double minutes = dms - (degrees * 100);
    double dd = degrees + (minutes / 60);
    return round(dd * 1e6) / 1e6;
}

SeapathData parse_nmea_data(std::string nmea_data) {
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
            std::cout << "N [m]: " << n << " \t E [m]: " << e << std::endl;
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
            seapath_data.cog_rad = std::round(std::stod(ground_course_rad) * 2 * pi / 360.0 * 1e6) / 1e6;
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


int main() {
    const char *UDP_IP = "0.0.0.0";
    const int UDP_PORT = 31420;

    int sockfd;
    char buffer[1024];
    struct sockaddr_in servaddr, cliaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    while(true) {
        int len = sizeof(cliaddr);
        int n = recvfrom(sockfd, (char *)buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *) &cliaddr, (socklen_t *)&len);
        buffer[n] = '\0';

        SeapathData seapath_data = parse_nmea_data(buffer);
        //std::cout << seapath_data.north << ", " << seapath_data.east << ", " << seapath_data.altitude << std::endl;
    }

    return 0;
}
