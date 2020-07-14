//
// Copyright 2020 i-RoCS
//

#include "sentinel_driver.h"
#include "msgs_defs.h"

#include <cstdio>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <sys/time.h>

#define FRAMESIZEMAX 275000

template <std::size_t N>
std::bitset<N> reverse(const std::bitset<N> &bit_set)
{
    std::bitset<N> reversed;
    for (unsigned int i = 0, j = N - 1; i < N; i++, j--)
    {
        reversed[j] = bit_set[i];
    }
    return reversed;
}

namespace Sentinel
{

SentinelLaser::SentinelLaser(std::shared_ptr<rclcpp::Node> _tmpNode)
{
    node = _tmpNode;
    configs.resize(4);
    
    // Default values
    selfIp = "192.168.1.10";
    lrfIp = "192.168.1.99";
    monitPort = 2000;
    laserEnabled[0] = true;
    laserEnabled[1] = false;
    laserEnabled[2] = false;
    laserEnabled[3] = false;

    intensityEnabled[0] = false;
    intensityEnabled[1] = false;
    intensityEnabled[2] = false;
    intensityEnabled[3] = false;

    configs[0].min_angle = 0;
    configs[0].max_angle = 275;
    configs[0].ang_increment = 0.1;
    configs[0].calcTimeIncrement();

    configs[1].min_angle = 0;
    configs[1].max_angle = 275;
    configs[1].ang_increment = 0.5;
    configs[1].calcTimeIncrement();

    configs[2].min_angle = 0;
    configs[2].max_angle = 275;
    configs[2].ang_increment = 0.5;
    configs[2].calcTimeIncrement();

    configs[3].min_angle = 0;
    configs[3].max_angle = 275;
    configs[3].ang_increment = 0.5;
    configs[3].calcTimeIncrement();

    configs[0].frame = "laser_master_frame";
    configs[1].frame = "laser_slave1_frame";
    configs[2].frame = "laser_slave2_frame";
    configs[3].frame = "laser_slave3_frame";

    configs[0].topic = "scan";
    configs[1].topic = "scan_2";
    configs[2].topic = "scan_3";
    configs[3].topic = "scan_4";

    //Declare Parameters

    node->declare_parameter("selfIp", selfIp);
    node->declare_parameter("lrfIp", lrfIp);
    node->declare_parameter("monitPort", monitPort);
    node->declare_parameter("masterEnabled", (bool)laserEnabled[0]);
    node->declare_parameter("slave1Enabled", (bool)laserEnabled[1]);
    node->declare_parameter("slave2Enabled", (bool)laserEnabled[2]);
    node->declare_parameter("slave3Enabled", (bool)laserEnabled[3]);

    node->declare_parameter("masterIntensityEnable", (bool)intensityEnabled[0]);
    node->declare_parameter("slave1IntensityEnable", (bool)intensityEnabled[1]);
    node->declare_parameter("slave2IntensityEnable", (bool)intensityEnabled[2]);
    node->declare_parameter("slave3IntensityEnable", (bool)intensityEnabled[3]);

    node->declare_parameter("masterMinAng", configs[0].min_angle);
    node->declare_parameter("masterMaxAng", configs[0].max_angle);
    node->declare_parameter("masterAngInc", configs[0].ang_increment);


    node->declare_parameter("slave1MinAng", configs[1].min_angle);
    node->declare_parameter("slave1MaxAng", configs[1].max_angle);
    node->declare_parameter("slave1AngInc", configs[1].ang_increment);

    node->declare_parameter("slave2MinAng", configs[2].min_angle);
    node->declare_parameter("slave2MaxAng", configs[2].max_angle);
    node->declare_parameter("slave2AngInc", configs[2].ang_increment);

    node->declare_parameter("slave3MinAng", configs[3].min_angle);
    node->declare_parameter("slave3MaxAng", configs[3].max_angle);
    node->declare_parameter("slave3AngInc", configs[3].ang_increment);

    node->declare_parameter("masterTfFrame", configs[0].frame);
    node->declare_parameter("slave1TfFrame", configs[1].frame);
    node->declare_parameter("slave2TfFrame", configs[2].frame);
    node->declare_parameter("slave3TfFrame", configs[3].frame);

    node->declare_parameter("masterTopic", configs[0].topic);
    node->declare_parameter("slave1Topic", configs[1].topic);
    node->declare_parameter("slave2Topic", configs[2].topic);
    node->declare_parameter("slave3Topic", configs[3].topic);

    readParameters();

    /* open device */
    const unsigned int long remoteIP = inet_addr(lrfIp.c_str()), port = 3000;
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);          //
    memset(&servaddr, 0, sizeof(struct sockaddr_in)); /* zero the struct */
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = remoteIP;
    servaddr.sin_port = htons(port);

    timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

SentinelLaser::~SentinelLaser()
{
    stopMonitoring();
    close(sockfd);
}

void SentinelLaser::readParameters()
{
    node->get_parameter("selfIp", selfIp);
    node->get_parameter("lrfIp", lrfIp);
    node->get_parameter("monitPort", monitPort);
    bool aux;
    node->get_parameter("masterEnabled", aux);
    laserEnabled[0] = aux;
    node->get_parameter("slave1Enabled", aux);
    laserEnabled[1] = aux;
    node->get_parameter("slave2Enabled", aux);
    laserEnabled[2] = aux;
    node->get_parameter("slave3Enabled", aux);
    laserEnabled[3] = aux;

    node->get_parameter("masterIntensityEnable", aux);
    intensityEnabled[0] = aux;
    node->get_parameter("slave1IntensityEnable", aux);
    intensityEnabled[1] = aux;
    node->get_parameter("slave2IntensityEnable", aux);
    intensityEnabled[2] = aux;
    node->get_parameter("slave3IntensityEnable", aux);
    intensityEnabled[3] = aux;

    node->get_parameter("masterMinAng", configs[0].min_angle);
    node->get_parameter("masterMaxAng", configs[0].max_angle);
    node->get_parameter("masterAngInc", configs[0].ang_increment);
    configs[0].calcTimeIncrement();

    node->get_parameter("slave1MinAng", configs[1].min_angle);
    node->get_parameter("slave1MaxAng", configs[1].max_angle);
    node->get_parameter("slave1AngInc", configs[1].ang_increment);
    configs[1].calcTimeIncrement();

    node->get_parameter("slave2MinAng", configs[2].min_angle);
    node->get_parameter("slave2MaxAng", configs[2].max_angle);
    node->get_parameter("slave2AngInc", configs[2].ang_increment);
    configs[2].calcTimeIncrement();

    node->get_parameter("slave3MinAng", configs[3].min_angle);
    node->get_parameter("slave3MaxAng", configs[3].max_angle);
    node->get_parameter("slave3AngInc", configs[3].ang_increment);
    configs[3].calcTimeIncrement();

    node->get_parameter("masterTfFrame", configs[0].frame);
    node->get_parameter("slave1TfFrame", configs[1].frame);
    node->get_parameter("slave2TfFrame", configs[2].frame);
    node->get_parameter("slave3TfFrame", configs[3].frame);

    node->get_parameter("masterTopic", configs[0].topic);
    node->get_parameter("slave1Topic", configs[1].topic);
    node->get_parameter("slave2Topic", configs[2].topic);
    node->get_parameter("slave3Topic", configs[3].topic);
}

bool SentinelLaser::reconfigure()
{
    stopMonitoring();

    startMonitoring();
    return false;
}

bool SentinelLaser::startMonitoring()
{
    start_msg start_monit;
    start_monit.ip = inet_addr(selfIp.c_str()); // self ip address
    start_monit.port = monitPort;
    start_monit.device_enabled = reverse(laserEnabled).to_ulong();
    start_monit.intensity_enabled = reverse(intensityEnabled).to_ulong();
    start_monit.point_in_safety_enabled = 0;
    start_monit.active_zone_set_enabled = 0;
    start_monit.io_pin_enabled = 0;
    start_monit.scan_counter_enabled = reverse(laserEnabled).to_ulong();
    start_monit.speed_encoder_enabled = 0;
    start_monit.diagnostics_enabled = 0;

    start_monit.master_start_angle = configs[0].min_angle * 10;
    start_monit.master_end_angle = configs[0].max_angle * 10;
    start_monit.master_resolution = configs[0].ang_increment * 10;
    start_monit.slave1_start_angle = configs[1].min_angle * 10;
    start_monit.slave1_end_angle = configs[1].max_angle * 10;
    start_monit.slave1_resolution = configs[1].ang_increment * 10;
    start_monit.slave2_start_angle = configs[2].min_angle * 10;
    start_monit.slave2_end_angle = configs[2].max_angle * 10;
    start_monit.slave2_resolution = configs[2].ang_increment * 10;
    start_monit.slave3_start_angle = configs[3].min_angle * 10;
    start_monit.slave3_end_angle = configs[3].max_angle * 10;
    start_monit.slave3_resolution = configs[3].ang_increment * 10;

    sendto(sockfd, start_monit.getMessage(), sizeof(start_monit), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

    //READ response
    reply_msg reply;
    socklen_t len = sizeof(servaddr);
    int size = recvfrom(sockfd, &reply, sizeof(reply), 0, (struct sockaddr *)&servaddr, &len);

    if (reply.res_code == 0 && size == 16)
        fprintf(stderr, "Start message accepted!!\n");

    return reply.res_code == 0;
}

bool SentinelLaser::stopMonitoring()
{
    end_msg stop_msg;
    sendto(sockfd, &stop_msg, sizeof(end_msg), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
    fprintf(stderr, "Stop message sent!!\n");

    //READ response
    reply_msg reply;
    socklen_t len = sizeof(servaddr);
    int size = recvfrom(sockfd, &reply, sizeof(reply), 0, (struct sockaddr *)&servaddr, &len);

    if (reply.res_code == 0 && size == 16)
        fprintf(stderr, "Stop message accepted!!\n");
    else
    {
        fprintf(stderr, "No response!!\n");
        return false;
    }

    return reply.res_code == 0;
}

void *SentinelLaser::receiveMsgs(void *arg)
{
    SentinelLaser *laser = (SentinelLaser *)arg;
    int sockfd;

    struct sockaddr_in monitSocket;
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&monitSocket, 0, sizeof(struct sockaddr_in)); /* zero the struct */
    monitSocket.sin_family = AF_INET;
    monitSocket.sin_addr.s_addr = htonl(INADDR_ANY);
    monitSocket.sin_port = htons(laser->monitPort);

    timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    bind(sockfd, (struct sockaddr *)&monitSocket, sizeof(monitSocket));

    unsigned char frame[FRAMESIZEMAX];
    socklen_t len = sizeof(monitSocket);

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,7),rmw_qos_profile_sensor_data);
    qos.keep_last(7);
    qos.reliable();

    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> laser_pub(4);
    for (unsigned int i = 0; i < 4; i++)
    {
        if (!laser->laserEnabled[i])
            continue;
        laser_pub[i] = laser->node->create_publisher<sensor_msgs::msg::LaserScan>(laser->configs[i].topic, qos);
    }
    std::vector<LaserScan> laserScans(4);
    std::vector<int> measuresSize(4, 0);
    for (unsigned int i = 0; i < laserScans.size(); i++)
    {
        laserScans[i].config = laser->configs[i];
        if (laserScans[i].config.ang_increment != 0.0)
            measuresSize[i] = (laserScans[i].config.max_angle - laserScans[i].config.min_angle) / laserScans[i].config.ang_increment;
    }

    std::vector<uint32_t> spinCount(4, 0);
    int scanner_id = -1;

    while (rclcpp::ok())
    {

        int size = recvfrom(sockfd, frame, FRAMESIZEMAX, 0, (struct sockaddr *)&monitSocket, &len);
        if (size < 21)
            continue;
        // fprintf(stderr,"%02x \n", frame[16]);
        // int i;
        // for (i = 0; i < 21; i++)
        // {
        //     fprintf(stderr, "%02x ", frame[i]);
        // }
        // for (; i < size; i++)
        // {
        //     if ((i % 16) == 5)
        //         fprintf(stderr, "\n");
        //     fprintf(stderr, "%02x ", frame[i]);
        // }
        // fprintf(stderr, "\n");
        // fprintf(stderr, "\n");

        monit_msg monitoring_msg;
        memcpy(&monitoring_msg, frame, 21);

        if (monitoring_msg.op_code != 0xCA)
            continue;

        struct timeval timeofday;
        gettimeofday(&timeofday, NULL);
        uint64_t currTime = (timeofday.tv_sec * 1000000000LL + timeofday.tv_usec * 1000);

        uint16_t offset = 21;
        bool endFrame = false;
        do
        {

            optional_inf optionalInf;
            memcpy(&optionalInf, frame + offset, 3);
            uint16_t lenght = optionalInf.lenght - 1;
            offset += 3;
            switch (optionalInf.header_id)
            {
            case 1: //io pin

                break;
            case 2: //scan counter
                scanner_id = monitoring_msg.scanner_id;
                uint32_t _spinCount;
                memcpy(&_spinCount, frame + offset, 4);
                if (_spinCount != spinCount[scanner_id])
                {
                    //Publish scan
                    if (spinCount[scanner_id] != 0)
                        publish(laser_pub[scanner_id], laserScans[scanner_id]);

                    spinCount[scanner_id] = _spinCount;

                    laserScans[scanner_id].ranges.clear();
                    laserScans[scanner_id].ranges.resize(measuresSize[scanner_id]);
                    if (laser->intensityEnabled[scanner_id])
                    {
                        laserScans[scanner_id].intensities.clear();
                        laserScans[scanner_id].intensities.resize(measuresSize[scanner_id]);
                    }
                    laserScans[scanner_id].scan_counter = _spinCount;
                    laserScans[scanner_id].system_time_stamp = currTime;
                }
                // fprintf(stderr,"spin count %d\r",_spinCount);

                break;
            case 3: //zone set

                break;
            case 4: //diagnostics

                break;
            case 5: //measures
                if (lenght > 0)
                {
                    uint32_t aux;
                    aux = (monitoring_msg.from_theta / 10 - laserScans[scanner_id].config.min_angle) / laserScans[scanner_id].config.ang_increment;
                    memcpy(&laserScans[scanner_id].ranges[aux], frame + offset, lenght);
                }
                break;
            case 6: //intensity
                if (lenght > 0)
                {
                    uint32_t aux;
                    aux = (monitoring_msg.from_theta / 10 - laserScans[scanner_id].config.min_angle) / laserScans[scanner_id].config.ang_increment;
                    memcpy(&laserScans[scanner_id].intensities[aux], frame + offset, lenght);
                }
                break;
            case 7: //encoder

                break;
            case 8: //point in safety

                break;
            case 9: //frame end
                endFrame = true;
                break;

            default:
                fprintf(stderr, "Unknown id. %d Invalid offset?? %d size: %d\n", frame[offset], offset, size);
                exit(1);
                break;
            }
            offset += lenght;

        } while (!endFrame);
    }

    close(sockfd);

    return NULL;
}

void SentinelLaser::publish(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub, LaserScan laserscan)
{
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    scan_msg->header.stamp.sec = RCL_NS_TO_S(laserscan.system_time_stamp);
    scan_msg->header.stamp.nanosec = laserscan.system_time_stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
    scan_msg->header.frame_id = laserscan.config.frame;
    scan_msg->angle_min = (laserscan.config.min_angle - 137.5) * M_PI / 180;
    scan_msg->angle_max = (laserscan.config.max_angle - 137.5 - laserscan.config.ang_increment) * M_PI / 180;
    scan_msg->angle_increment = laserscan.config.ang_increment * M_PI / 180;
    scan_msg->scan_time = laserscan.config.scan_time;
    scan_msg->time_increment = laserscan.config.time_increment;
    scan_msg->range_min = laserscan.config.min_range;
    scan_msg->range_max = laserscan.config.max_range;

    for (unsigned int i = 0; i < laserscan.ranges.size(); i++)
    {
        scan_msg->ranges.push_back(laserscan.ranges[i] / 1000.0);
    }
    //Intensities 
    //FIXME Device specific units 
    for (unsigned int i = 0; i < laserscan.intensities.size(); i++)
    {
        scan_msg->intensities.push_back(laserscan.intensities[i]);
    }
    laser_pub->publish(*scan_msg);
}

} // namespace Sentinel