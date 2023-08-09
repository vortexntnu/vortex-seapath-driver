#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <string>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

class SensorData {
public:
    double roll;
    double pitch;
    double yaw;
    double angular_rate_roll;
    double angular_rate_pitch;
    double angular_rate_yaw;
    double lin_acc_x;
    double lin_acc_y;
    double lin_acc_z;

    SensorData(double r, double p, double y, double arr, double arp, double ary, double lax, double lay, double laz)
    : roll(r), pitch(p), yaw(y), angular_rate_roll(arr), angular_rate_pitch(arp), angular_rate_yaw(ary), lin_acc_x(lax), lin_acc_y(lay), lin_acc_z(laz) {}
};

SensorData receiveNMEA(int sockfd, struct sockaddr_in& cliaddr);
SensorData parseNMEA(std::string str);


#endif // SENSOR_DATA_H
