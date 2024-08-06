// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifndef ORD_LIDAR_DRIVER_H
#define ORD_LIDAR_DRIVER_H

#include <thread>

#include "ordlidar_protocol.h"
#include <atomic>
#include <core/base/locker.h>
#include <core/base/thread.h>
#include <core/base/timer.h>
#include <core/common/ChannelDevice.h>
#include <map>
#include <stdlib.h>
#include <vector>

namespace ordlidar {
using namespace core;
using namespace base;
using namespace core::common;
class OrdlidarDriver
{
public:
  OrdlidarDriver(uint8_t type, int model);
  ~OrdlidarDriver();

  bool SetSerialPort(const std::string &port_name, const uint32_t &baudrate);
  bool Connect();
  void Disconnect();
  bool isConnected() const;
  bool Activate();
  bool Deactive();
  bool GrabOneScan(one_scan_data_st &scan_data);
  bool GrabOneScanBlocking(one_scan_data_st &scan_data, int timeout_ms);
  bool GrabFullScan(full_scan_data_st &scan_data);
  bool GrabFullScanBlocking(full_scan_data_st &scan_data, int timeout_ms);
  uint16_t GetTimestamp() const;
  double GetRotationSpeed() const;
  bool SetRotationSpeed(int speed);
  bool GetFirmwareVersion(std::string &top_fw_version, std::string &bot_fw_version);
  bool GetDeviceSN(std::string &device_sn);

private:
  static void mRxThreadProc(void *arg);
  int read(unsigned char *data, int length);
  int write(unsigned char *data, int length);
  bool uart_data_handle(unsigned char *data, int len);
  bool uart_data_find_init_info(unsigned char *data, int len);
  bool IsFullScanReady(void) { return full_scan_ready_; }
  void ResetFullScanReady(void) { full_scan_ready_ = false; }
  bool IsOneScanReady(void) { return one_scan_ready_; }
  void ResetOneScanReady(void) { one_scan_ready_ = false; }
  // int point_data_parse_frame_ms200(point_data_t *data, unsigned char *buf, unsigned short buf_len, float start_angle,
  // float end_angle);
  int point_data_parse_frame_ms200(point_data_t *data, OradarLidarFrame *pkg);

private:
  // serial port
  ChannelDevice *serial_;
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
  int valid_data_;
  uint8_t init_info_flag_;
  std::vector<uint8_t> bin_buf_;
  std::vector<uint8_t> cmd_buf_;
  parsed_data_st parsed_data_;
  full_scan_data_st full_scan_data_;
  full_scan_data_st temp_data;
  one_scan_data_st one_scan_data_;
  std::thread *rx_thread_;
  std::atomic<bool> rx_thread_exit_flag_;

  Event full_data_event_;
  Event one_data_event_;
  // Locker lock_;
};

}// namespace ordlidar

#endif
