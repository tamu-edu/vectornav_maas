/** VectorNav ROS2 antenna-offset utility
 *
 * Utility to set / dump the GPS Antenna Offset and GPS Compass Baseline
 * registers on a VectorNav sensor from ROS2 parameters (a YAML config
 * file). Antenna offset is available on GPS-capable devices
 * (VN-200/VN-300); the compass baseline is a VN-300 dual-antenna feature.
 *
 * Copyright 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#if __linux__ || __CYGWIN__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#include <rclcpp/rclcpp.hpp>

#include "vn/sensors.h"
#include "vn/util.h"

// Assure that the serial port is set to async low latency in order to reduce
// delays and package pileup. These changes stay effective until the device is
// unplugged. (Mirrors the helper in vectornav.cc.)
#if __linux__ || __CYGWIN__
static bool optimize_serial_communication(
  const rclcpp::Logger & logger, const std::string & portName)
{
  int portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);
  if (portFd == -1) {
    RCLCPP_WARN(logger, "Can't open port for optimization");
    return false;
  }
  RCLCPP_INFO(logger, "Set port to ASYNC_LOW_LATENCY");
  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  ::close(portFd);
  return true;
}
#else
static bool optimize_serial_communication(
  const rclcpp::Logger & logger, const std::string & portName)
{
  (void)logger;
  (void)portName;
  return true;
}
#endif

// Validate a 3-element double array into a vec3f.
static bool toVec3f(const std::vector<double> & arr, vn::math::vec3f & out)
{
  if (arr.size() != 3) {
    return false;
  }
  for (int i = 0; i < 3; i++) {
    out[i] = static_cast<float>(arr[i]);
  }
  return true;
}

static void printVec3f(
  const rclcpp::Logger & logger, const std::string & label, const vn::math::vec3f & v)
{
  RCLCPP_INFO(logger, "%s: [%f, %f, %f]", label.c_str(), v[0], v[1], v[2]);
}

// Connect to the sensor, scanning supported baud rates (mirrors vectornav.cc).
static bool connectSensor(
  const rclcpp::Logger & logger, vn::sensors::VnSensor & vs, const std::string & SensorPort,
  int SensorBaudrate)
{
  optimize_serial_communication(logger, SensorPort);
  RCLCPP_INFO(logger, "Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

  std::vector<unsigned int> supportedBaudrates = vs.supportedBaudrates();
  supportedBaudrates.insert(supportedBaudrates.begin(), static_cast<unsigned int>(SensorBaudrate));

  bool baudSet = false;
  int i = 0;
  while (!baudSet) {
    unsigned int defaultBaudrate = supportedBaudrates[i];
    RCLCPP_INFO(logger, "Connecting with default at %d", defaultBaudrate);
    // Default response was too low and retransmit time was too long by default.
    vs.setResponseTimeoutMs(1000);  // Wait for up to 1000 ms for response
    vs.setRetransmitDelayMs(50);    // Retransmit every 50 ms

    try {
      // 128000 is not a valid baud rate on the VN100; skip it.
      if (defaultBaudrate != 128000 && SensorBaudrate != 128000) {
        vs.connect(SensorPort, defaultBaudrate);
        // Change the sensor's baudrate and reconnect the serial port at the new rate.
        vs.changeBaudRate(SensorBaudrate);
        RCLCPP_INFO(logger, "Connected baud rate is %d", vs.baudrate());
        baudSet = true;
      }
    } catch (...) {
      // Disconnect if we had the wrong default and were connected.
      vs.disconnect();
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    i++;
    // There are only 9 available data rates; if no connection made yet,
    // possibly a hardware malfunction.
    if (i > 8) {
      break;
    }
  }
  return baudSet;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vn_set_antenna_config");

  // Connection parameters.
  auto SensorPort = node->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  auto SensorBaudrate = node->declare_parameter<int>("serial_baud", 115200);
  auto read_only = node->declare_parameter<bool>("read_only", false);

  // Optional antenna offset / baseline values. An empty array means "skip".
  node->declare_parameter<std::vector<double>>("gps_antenna_offset", std::vector<double>{});
  node->declare_parameter<std::vector<double>>(
    "gps_compass_baseline.position", std::vector<double>{});
  node->declare_parameter<std::vector<double>>(
    "gps_compass_baseline.uncertainty", std::vector<double>{});

  const rclcpp::Logger logger = node->get_logger();

  // Retrieve and validate the optional vec3 parameters.
  bool has_antenna_offset = false;
  vn::math::vec3f antenna_offset;
  {
    auto arr = node->get_parameter("gps_antenna_offset").as_double_array();
    if (!arr.empty()) {
      if (!toVec3f(arr, antenna_offset)) {
        RCLCPP_ERROR(logger, "gps_antenna_offset must be a 3-element numeric array");
        rclcpp::shutdown();
        return 1;
      }
      has_antenna_offset = true;
    }
  }

  bool has_baseline = false;
  vn::math::vec3f baseline_position;
  vn::math::vec3f baseline_uncertainty;
  {
    auto pos = node->get_parameter("gps_compass_baseline.position").as_double_array();
    auto unc = node->get_parameter("gps_compass_baseline.uncertainty").as_double_array();
    if (!pos.empty()) {
      if (!toVec3f(pos, baseline_position)) {
        RCLCPP_ERROR(
          logger, "gps_compass_baseline.position must be a 3-element numeric array");
        rclcpp::shutdown();
        return 1;
      }
      if (!toVec3f(unc, baseline_uncertainty)) {
        RCLCPP_ERROR(
          logger, "gps_compass_baseline.uncertainty must be a 3-element numeric array");
        rclcpp::shutdown();
        return 1;
      }
      has_baseline = true;
    } else if (!unc.empty()) {
      RCLCPP_ERROR(
        logger, "gps_compass_baseline.uncertainty set without gps_compass_baseline.position");
      rclcpp::shutdown();
      return 1;
    }
  }

  // Connect to the sensor.
  vn::sensors::VnSensor vs;
  if (!connectSensor(logger, vs, SensorPort, SensorBaudrate)) {
    RCLCPP_ERROR(logger, "Failed to connect to sensor on %s", SensorPort.c_str());
    rclcpp::shutdown();
    return 1;
  }
  if (!vs.verifySensorConnectivity()) {
    RCLCPP_ERROR(logger, "No device communication");
    vs.disconnect();
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "Device connection established");
  std::string mn = vs.readModelNumber();
  std::string fv = vs.readFirmwareVersion();
  RCLCPP_INFO(logger, "Model Number: %s, Firmware Version: %s", mn.c_str(), fv.c_str());

  // Determine device capabilities from the model number string. The
  // vnproglib Family enum predates the VN-310 (a dual-antenna GNSS/INS like
  // the VN-300) and reports it as Family_Unknown, so we can't rely on it.
  bool is_vn100 = (mn.find("VN-100") == 0);
  bool is_dual_antenna = (mn.find("VN-3") == 0);  // VN-300, VN-310

  if (read_only) {
    RCLCPP_INFO(logger, "=== Read-only mode: dumping current register values ===");

    if (is_vn100) {
      RCLCPP_WARN(logger, "GpsAntennaOffset is not available on the VN-100");
    } else {
      try {
        vn::math::vec3f ao = vs.readGpsAntennaOffset();
        printVec3f(logger, "GPS Antenna Offset", ao);
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger, "Failed reading GpsAntennaOffset: %s", e.what());
      }
    }

    if (!is_dual_antenna) {
      RCLCPP_WARN(
        logger,
        "GpsCompassBaseline is only available on dual-antenna devices (VN-300/VN-310)");
    } else {
      try {
        vn::sensors::GpsCompassBaselineRegister bl = vs.readGpsCompassBaseline();
        printVec3f(logger, "GPS Compass Baseline Position", bl.position);
        printVec3f(logger, "GPS Compass Baseline Uncertainty", bl.uncertainty);
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger, "Failed reading GpsCompassBaseline: %s", e.what());
      }
      try {
        vn::sensors::GpsCompassEstimatedBaselineRegister eb = vs.readGpsCompassEstimatedBaseline();
        printVec3f(logger, "GPS Compass Estimated Baseline Position", eb.position);
        printVec3f(logger, "GPS Compass Estimated Baseline Uncertainty", eb.uncertainty);
        RCLCPP_INFO(
          logger, "estBaselineUsed: %s, numMeas: %d", eb.estBaselineUsed ? "true" : "false",
          eb.numMeas);
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger, "Failed reading GpsCompassEstimatedBaseline: %s", e.what());
      }
    }
  } else {
    if (!has_antenna_offset && !has_baseline) {
      RCLCPP_WARN(
        logger,
        "No antenna offset or baseline specified. Set gps_antenna_offset and/or "
        "gps_compass_baseline, or use read_only: true to dump current values.");
    }

    if (has_antenna_offset) {
      if (is_vn100) {
        RCLCPP_WARN(logger, "GpsAntennaOffset is not available on the VN-100; skipping");
      } else {
        RCLCPP_INFO(logger, "Writing GPS Antenna Offset...");
        try {
          vs.writeGpsAntennaOffset(antenna_offset);
          vn::math::vec3f ao = vs.readGpsAntennaOffset();
          printVec3f(logger, "Readback GPS Antenna Offset", ao);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(logger, "Failed writing GpsAntennaOffset: %s", e.what());
        }
      }
    }

    if (has_baseline) {
      if (!is_dual_antenna) {
        RCLCPP_WARN(
          logger,
          "GpsCompassBaseline is only available on dual-antenna devices "
          "(VN-300/VN-310); skipping");
      } else {
        RCLCPP_INFO(logger, "Writing GPS Compass Baseline...");
        try {
          vs.writeGpsCompassBaseline(baseline_position, baseline_uncertainty);
          vn::sensors::GpsCompassBaselineRegister bl = vs.readGpsCompassBaseline();
          printVec3f(logger, "Readback GPS Compass Baseline Position", bl.position);
          printVec3f(logger, "Readback GPS Compass Baseline Uncertainty", bl.uncertainty);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(logger, "Failed writing GpsCompassBaseline: %s", e.what());
        }
      }
    }
  }

  vs.disconnect();
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  RCLCPP_INFO(logger, "Disconnected successfully");
  rclcpp::shutdown();
  return 0;
}
