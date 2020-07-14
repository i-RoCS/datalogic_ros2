//
// Copyright 2020 i-RoCS
//

#include <math.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <unistd.h>
#include <thread>

#include "sentinel_driver.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sentinel_node");

  Sentinel::SentinelLaser laser(node);

  laser.stopMonitoring();
  fprintf(stderr, "[SENTINEL INFO] Starting Monitoring\n");
  if (!laser.startMonitoring())
  {
    fprintf(stderr, "INVALID CONFIG!!\n");
    exit(1);
  }

  std::thread t1(&Sentinel::SentinelLaser::receiveMsgs, (void *)&laser);

  fprintf(stdout, "Running!!\n");
  rclcpp::WallRate loop_rate(10);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  fprintf(stderr, "[SENTINEL INFO] Now SENTINEL is stopping .......\n");

  t1.join();

  laser.stopMonitoring();
  rclcpp::shutdown();

  return 0;
}
