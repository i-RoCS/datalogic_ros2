# Datalogic Sentinel Laser Monitoring ros2

## Overview
This package provides an implementation of Laser Sentinel Advanced Monitoring protocol in order to obtain and publish the laser scan measures.
All options can be configured via parameters.

### License

Apache 2.0

**Author: Filipe Amaral, f.amaral@ua.pt**

The package as been tested under ROS2 Dashing Diademata and Ubuntu 18.04.

## Nodes

### sentinel_node

#### Parameters

* **`lrfIp`**: Ip address of master/standalone. Default: `"192.168.1.99"`
* **`selfIp`**: Ip address of machine running the node. Default: `"192.168.1.10"`
* **`monitPort`**: Port number for monitoring. Default: `2000`
* **`masterEnabled`**: Enable monitoring of master lrf. Default: `true`
* **`masterIntensityEnable`**: Enable intesities monitoring for master lrf. Default: `false`
* **`masterMinAng`**: Master lrf initial monitoring angle. Default: `0.0`
* **`masterMaxAng`**: Master lrf final monitoring angle. Default: `275.0`
* **`masterAngInc`**: Master lrf angular resolution. Default: `0.1`
* **`masterTfFrame`**: Master lrf Tf frame. Default: `"laser_master_frame"`
* **`masterTopic`**: LaserScan topic for master lrf. Default: `"scan"`
* **`slave1Enabled`**: Enable monitoring of slave1 lrf. Default: `false`
* **`slave1IntensityEnable`**:  Enable intesities monitoring for slave1 lrf. Default: `false`
* **`slave1MinAng`**: Slave1 lrf initial monitoring angle. Default: `0.0`
* **`slave1MaxAng`**: Slave1 lrf final monitoring angle. Default: `275.0`
* **`slave1AngInc`**: Slave1 lrf angular resolution. Default: `0.5`
* **`slave1TfFrame`**: Slave1 lrf Tf frame. Default: `"laser_slave1_frame"`
* **`slave1Topic`**: LaserScan topic for slave1 lrf. Default: `"scan_2"`
* **`slave2Enabled`**: Enable monitoring of slave2 lrf. Default: `false`
* **`slave2IntensityEnable`**: Enable intesities monitoring for slave2 lrf. Default: `false`
* **`slave2MinAng`**: Slave2 lrf initial monitoring angle. Default: `0.0`
* **`slave2MaxAng`**: Slave2 lrf final monitoring angle. Default: `275.0`
* **`slave2AngInc`**: Slave2 lrf angular resolution. Default: `0.5`
* **`slave2TfFrame`**: Slave2 lrf Tf frame. Default: `"laser_slave2_frame"`
* **`slave2Topic`**: LaserScan topic for slave2 lrf. Default: `"scan_3"`
* **`slave3Enabled`**: Enable monitoring of slave3 lrf. Default: `false`
* **`slave3IntensityEnable`**: Enable intesities monitoring for slave3 lrf. Default: `false`
* **`slave3MinAng`**: Slave3 lrf initial monitoring angle. Default: `0.0`
* **`slave3MaxAng`**: Slave3 lrf final monitoring angle. Default: `275.0`
* **`slave3AngInc`**: Slave3 lrf angular resolution. Default: `0.5`
* **`slave3TfFrame`**: Slave3 lrf Tf frame. Default: `"laser_slave3_frame"`
* **`slave3Topic`**: LaserScan topic for slave3 lrf. Default: `"scan_4"`

#### Published Topics

* **`scan`** ([sensor_msgs/LaserScan]) 
    One topic per active laser. The topic name can specified via parameter.

## Ackowledgments

This module was developed in context of the Project i-RoCS (POCI-01-0247-FEDER-039947).
