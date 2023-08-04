#include <iostream>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

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
    
    void print() {
        std::cout << "Roll: " << roll << "\n"
                  << "Pitch: " << pitch << "\n"
                  << "Yaw: " << yaw << "\n"
                  << "Angular Rate Roll: " << angular_rate_roll << "\n"
                  << "Angular Rate Pitch: " << angular_rate_pitch << "\n"
                  << "Angular Rate Yaw: " << angular_rate_yaw << "\n"
                  << "Linear Acceleration Roll: " << lin_acc_roll << "\n"
                  << "Linear Acceleration Pitch: " << lin_acc_pitch << "\n"
                  << "Linear Acceleration Yaw: " << lin_acc_yaw << std::endl;
    }
};

SensorData parseNMEA(std::string str) {
    std::istringstream ss(str);
    std::string token;
    
    // Skip the first three tokens
    for (int i = 0; i < 3; ++i) {
        std::getline(ss, token, ',');
    }

    double data[9];
    for (int i = 0; i < 9; ++i) {
        std::getline(ss, token, ',');
        data[i] = std::stod(token);
    }

    return SensorData(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
}

int main() {
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return 1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(7552);

    // Bind the socket
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind failed");
        return 1;
    }

    while (true) {
        char buffer[1024];
        socklen_t len = sizeof(cliaddr);

        int n = recvfrom(sockfd, (char *)buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr*)&cliaddr, &len);
        buffer[n] = '\0';  // ensure null-terminated string

        SensorData data = parseNMEA(buffer);
        data.print();
    }

    close(sockfd);

    return 0;
}
