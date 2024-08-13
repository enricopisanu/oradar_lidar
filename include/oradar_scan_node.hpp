#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "src/ord_lidar_driver.h"


class OradarScanNode : public rclcpp::Node
{
public:
  explicit OradarScanNode(std::string node_name);
  ~OradarScanNode() { device_.disconnect(); }
  OradarScanNode(const OradarScanNode &other) = default;
  OradarScanNode &operator=(const OradarScanNode &other) = default;
  OradarScanNode(OradarScanNode &&other) noexcept = default;
  OradarScanNode &operator=(OradarScanNode &&other) noexcept = default;


private:
  void declareParameters();
  [[nodiscard]] bool connectToLidar();
  void publishScan(full_scan_data_st *scan_frame, rclcpp::Time start_time, double scan_time);
  void scanAndPublish();

  [[nodiscard]] double degToRad(const double deg);

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;


  ordlidar::OrdlidarDriver device_;

  std::string frame_id_;
  std::string scan_topic_;
  std::string port_;
  std::string device_model_;
  int baudrate_ = 230400;
  int motor_speed_ = 10;
  double angle_min_ = 0.0;
  double angle_max_ = 360.0;
  double min_range_ = 0.05;
  double max_range_ = 20.0;
  bool clockwise_ = false;
};
