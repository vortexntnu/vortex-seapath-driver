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
    double lin_acc_roll;
    double lin_acc_pitch;
    double lin_acc_yaw;

    SensorData(double r, double p, double y, double arr, double arp, double ary, double lar, double lap, double lay)
    : roll(r), pitch(p), yaw(y), angular_rate_roll(arr), angular_rate_pitch(arp), angular_rate_yaw(ary), lin_acc_roll(lar), lin_acc_pitch(lap), lin_acc_yaw(lay) {}
};

SensorData receiveNMEA(int sockfd, struct sockaddr_in& cliaddr);
SensorData parseNMEA(std::string str);


#endif // SENSOR_DATA_H
