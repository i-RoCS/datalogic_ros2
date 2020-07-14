//
// Copyright 2020 i-RoCS
//

#ifndef SENTINEL_AUX
#define SENTINEL_AUX

#include <vector>
#include <stdint.h>
#include <string>
namespace Sentinel
{

//! A struct for returning configuration from the DlSentinel
struct LaserConfig
{
  //! Start angle for the laser scan [deg].
  float min_angle;
  //! Stop angle for the laser scan [deg].
  float max_angle;
  //! Scan resolution [deg].
  float ang_increment;
  //! Scan resoltuion [s]
  float time_increment;
  //! Time between scans
  float scan_time = 0.030;
  //! Minimum range [m]
  float min_range = 0.050;
  //! Maximum range [m]
  float max_range = 40;

  void calcTimeIncrement()
  {
    time_increment = scan_time / 360 * ang_increment;
  }
  std::string frame;
  std::string topic;
};

//! A struct for returning laser readings from the DlSentinel
struct LaserScan
{
  //! Array of ranges
  std::vector<uint16_t> ranges;
  //! Array of intensities
  std::vector<uint16_t> intensities;
  //! Spins since start
  uint32_t scan_counter;
  //! System time when first range was measured in nanoseconds
  uint64_t system_time_stamp;
  //! Configuration of scan
  LaserConfig config;
};
} // namespace Sentinel

#endif