// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifndef ORD_LIDAR_DRIVER_H
#define ORD_LIDAR_DRIVER_H

#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <map>
#include <thread>
#include <vector>

#include "ordlidar_protocol.h"
#include <core/base/locker.h>
#include <core/base/thread.h>
#include <core/base/timer.h>
#include <core/base/utils.h>
#include <core/common/ChannelDevice.h>
#include <core/network/ActiveSocket.h>
#include <core/serial/common.h>
#include <core/serial/serial.h>
#include <stdlib.h>

namespace ordlidar {
using namespace core;
using namespace base;
using namespace core::common;
class OrdlidarDriver
{
public:
  OrdlidarDriver(uint8_t type, int model);
  ~OrdlidarDriver();

  auto setSerialPort(const std::string &port_name, uint32_t baudrate) -> void;
  [[nodiscard]] auto connect() -> bool;
  auto disconnect() -> void;
  [[nodiscard]] auto isConnected() const noexcept -> bool;
  [[nodiscard]] auto activate() -> bool;
  [[nodiscard]] auto deactive() -> bool;
  [[nodiscard]] auto grabOneScan(one_scan_data_st &scan_data) -> bool;
  [[nodiscard]] auto grabOneScanBlocking(one_scan_data_st &scan_data, int timeout_ms) -> bool;
  [[nodiscard]] auto grabFullScan(full_scan_data_st &scan_data) -> bool;
  [[nodiscard]] auto grabFullScanBlocking(full_scan_data_st &scan_data, int timeout_ms) -> bool;
  [[nodiscard]] auto getTimestamp() const -> uint16_t;
  [[nodiscard]] auto getRotationSpeed() const -> double;
  [[nodiscard]] auto setRotationSpeed(int speed) -> bool;
  [[nodiscard]] auto getFirmwareVersion(std::string &top_fw_version, std::string &bot_fw_version) -> bool;
  [[nodiscard]] auto getDeviceSN(std::string &device_sn) -> bool;

private:
  static void mRxThreadProc(OrdlidarDriver &lidar_obj);
  // int read(unsigned char *data, int length);
  [[nodiscard]] auto write(unsigned char *data, int length) -> int;
  [[nodiscard]] auto uartDataHandle(unsigned char *data, int len) -> bool;
  auto uartDataFindInitInfo(unsigned char *data, int len) -> void;
  [[nodiscard]] auto isFullScanReady() -> bool { return full_scan_ready_; }
  auto resetFullScanReady() -> void { full_scan_ready_ = false; }
  [[nodiscard]] auto isOneScanReady() -> bool { return one_scan_ready_; }
  auto resetOneScanReady() -> void { one_scan_ready_ = false; }
  // int point_data_parse_frame_ms200(point_data_t *data, unsigned char *buf, unsigned short buf_len, float start_angle,
  // float end_angle);
 auto pointDataParseFrameMs200(point_data_t *data, OradarLidarFrame *pkg) -> void;

  static constexpr int SHORT_DELAY_MS = 200;
  static constexpr int MEDIUM_DELAY_MS = 400;
  static constexpr int LONG_DELAY_MS = 1000;

  // serial port
  std::unique_ptr<ChannelDevice> serial_;

  std::string port_name_;
  std::string top_fw_version_;
  std::string bot_fw_version_;
  std::string device_sn_;
  uint32_t baudrate_;
  // tranformer type
  uint8_t tranformer_type_;
  int model_;
  bool is_connected_;
  bool full_scan_ready_;
  bool one_scan_ready_;
  bool valid_data_;
  uint8_t init_info_flag_;
  std::vector<uint8_t> bin_buf_;
  std::vector<uint8_t> cmd_buf_;
  parsed_data_st parsed_data_;
  full_scan_data_st full_scan_data_;
  full_scan_data_st temp_data;
  one_scan_data_st one_scan_data_;
  std::unique_ptr<std::thread> rx_thread_;
  std::atomic<bool> rx_thread_exit_flag_;

  Event full_data_event_;
  Event one_data_event_;
  // Locker lock_;
};

}// namespace ordlidar

#endif
