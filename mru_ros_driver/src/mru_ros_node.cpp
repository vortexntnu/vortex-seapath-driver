#include "mru_ros_driver/mru_socket.hpp"
#include "mru_ros_driver/mru_ros_driver.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mru_ros_driver");

    MRURosDriver driver;

    int sockfd;
    struct sockaddr_in servaddr, cliaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR("Socket creation failed");
        return 1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(7552);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        ROS_ERROR("bind failed");
        return 1;
    }

    while (ros::ok()) {
        SensorData data = receiveNMEA(sockfd, cliaddr);
        driver.publishData(data);
        ros::spinOnce();
    }

    close(sockfd);

    return 0;
}
