//
// Copyright 2020 i-RoCS
//

#include <netinet/in.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sentinel_aux.h"
#include <bitset>

namespace Sentinel
{

class SentinelLaser
{
private:
    int sockfd;
    struct sockaddr_in servaddr, ReciveBoard;
    std::shared_ptr<rclcpp::Node> node;
    std::vector<LaserConfig> configs;
    std::bitset<4> intensityEnabled;
    std::bitset<4> laserEnabled;
    int monitPort;
    std::string selfIp;
    std::string lrfIp;
    static void publish(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub, LaserScan laserscan);
    void readParameters();
public:
    SentinelLaser(std::shared_ptr<rclcpp::Node> _tmpNode);
    ~SentinelLaser();

    bool reconfigure();
    bool startMonitoring();
    bool stopMonitoring();
    static void *receiveMsgs( void *laser);
};

} // namespace Sentinel
