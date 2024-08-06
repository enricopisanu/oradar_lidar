// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#include "oradar_scan_node.hpp"

OradarScanNode::OradarScanNode(std::string node_name)
  : Node(std::move(node_name)), device_(ORADAR_TYPE_SERIAL, ORADAR_MS200)
{
  declareParameters();

  if (!connectToLidar()) { return; }

  double min_threshold = static_cast<double>(motor_speed_) - (static_cast<double>(motor_speed_) * 0.1);
  double max_threshold = static_cast<double>(motor_speed_) + (static_cast<double>(motor_speed_) * 0.1);
  double current_speed = device_.GetRotationSpeed();
  if (current_speed < min_threshold || current_speed > max_threshold) { device_.SetRotationSpeed(motor_speed_); }


  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {});
}

void OradarScanNode::declareParameters()
{
  this->declare_parameter<std::string>("port_name", port_);
  this->declare_parameter<int>("baudrate", baudrate_);
  this->declare_parameter<double>("angle_max", angle_max_);
  this->declare_parameter<double>("angle_min", angle_min_);
  this->declare_parameter<double>("range_max", max_range_);
  this->declare_parameter<double>("range_min", min_range_);
  this->declare_parameter<bool>("clockwise", clockwise_);
  this->declare_parameter<int>("motor_speed", motor_speed_);
  this->declare_parameter<std::string>("device_model", device_model_);
  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<std::string>("scan_topic", scan_topic_);

  this->get_parameter("port_name", port_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("angle_max", angle_max_);
  this->get_parameter("angle_min", angle_min_);
  this->get_parameter("range_max", max_range_);
  this->get_parameter("range_min", min_range_);
  this->get_parameter("clockwise", clockwise_);
  this->get_parameter("motor_speed", motor_speed_);
  this->get_parameter("device_model", device_model_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("scan_topic", scan_topic_);
}

bool OradarScanNode::connectToLidar()
{
  if (port_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Can't find lidar MS200 \n");
    return false;
  }

  device_.SetSerialPort(port_, baudrate_);

  RCLCPP_INFO(this->get_logger(), "Get lidar type: %s \n", device_model_.c_str());
  RCLCPP_INFO(this->get_logger(), "Get serial port: %s, baudrate: %d \n", port_.c_str(), baudrate_);

  while (rclcpp::ok()) {
    if (device_.isConnected()) {
      device_.Disconnect();
      RCLCPP_INFO(this->get_logger(), "Disconnect lidar _. \n");
    }

    if (device_.Connect()) {
      RCLCPP_INFO(this->get_logger(),"Lidar device connected successfully. \n");
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(),"Lidar device connecting... \n");
      sleep(1);
    }
  }
  return false;
}

void OradarScanNode::scanAndPublish()
{

  full_scan_data_st scan_data;
  rclcpp::Time start_scan_time;
  rclcpp::Time end_scan_time;
  double scan_duration;
  bool grab;

  while (rclcpp::ok()) {
    start_scan_time = this->now();
    grab = device_.GrabFullScanBlocking(scan_data, 1000);
    scan_duration = (end_scan_time.seconds() - start_scan_time.seconds());
    if (grab) { publishScan(&scan_data, start_scan_time, scan_duration); }
  }
}
double OradarScanNode::degToRad(const double deg) { return deg * M_PI / 180.0; }

void OradarScanNode::publishScan(full_scan_data_st *scan_frame, rclcpp::Time start_time, double scan_time)
{
  auto scan_msg = sensor_msgs::msg::LaserScan();
  int point_nums = scan_frame->vailtidy_point_num;

  scan_msg.header.stamp = start_time;
  scan_msg.header.frame_id = frame_id_;
  scan_msg.angle_min = degToRad(scan_frame->data[0].angle);
  scan_msg.angle_max = degToRad(scan_frame->data[point_nums - 1].angle);
  double diff = scan_frame->data[point_nums - 1].angle - scan_frame->data[0].angle;
  scan_msg.angle_increment = degToRad(diff / point_nums);
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / point_nums;
  scan_msg.range_min = min_range_;
  scan_msg.range_max = max_range_;

  scan_msg.ranges.assign(point_nums, std::numeric_limits<float>::quiet_NaN());
  scan_msg.intensities.assign(point_nums, std::numeric_limits<float>::quiet_NaN());

  double range = 0.0;
  double intensity = 0.0;
  double dir_angle;
  uint32_t last_index = 0;

  for (size_t i = 0; i < point_nums; i++) {
    range = scan_frame->data[i].distance * 0.001;
    intensity = scan_frame->data[i].intensity;

    if ((range > max_range_) || (range < min_range_)) {
      range = 0.0;
      intensity = 0.0;
    }

    if (!clockwise_) {
      dir_angle = static_cast<double>(360.0 - scan_frame->data[i].angle);
    } else {
      dir_angle = scan_frame->data[i].angle;
    }

    if ((dir_angle < angle_min_) || (dir_angle > angle_max_)) {
      range = 0;
      intensity = 0;
    }

    double angle = degToRad(dir_angle);
    uint32_t index = static_cast<uint32_t>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
    if (index < point_nums) {
      // If the current content is Nan, it is assigned directly
      if (std::isnan(scan_msg.ranges[index])) {
        scan_msg.ranges[index] = range;
        uint32_t err = index - last_index;
        if (err == 2) {
          scan_msg.ranges[index - 1] = range;
          scan_msg.intensities[index - 1] = intensity;
        }
      } else {// Otherwise, only when the distance is less than the current
        //   value, it can be re assigned
        if (range < scan_msg.ranges[index]) { scan_msg.ranges[index] = range; }
      }
      scan_msg.intensities[index] = intensity;
      last_index = index;
    }
  }

  scan_pub_->publish(scan_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto lidar_node = std::make_shared<OradarScanNode>("oradar_ros");
  rclcpp::spin(lidar_node);
  rclcpp::shutdown();

  return 0;
}
