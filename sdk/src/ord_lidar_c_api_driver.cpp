// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#include "ord_lidar_c_api_driver.h"
#include "ord_lidar_driver.h"

using namespace ordlidar;

ORDLidar *oradar_lidar_create(uint8_t type, int model)
{
  OrdlidarDriver *driver = new OrdlidarDriver(type, model);
  ORDLidar *instance = new ORDLidar;
  instance->lidar = nullptr;
  instance->lidar = (void *)driver;
  return instance;
}

void oradar_lidar_destroy(ORDLidar **lidar)
{
  if (lidar == nullptr || *lidar == nullptr) { return; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>((*lidar)->lidar);

  if (drv) {
    delete drv;
    drv = nullptr;
  }

  (*lidar)->lidar = nullptr;
  delete *lidar;
  *lidar = nullptr;
  return;
}

bool oradar_set_serial_port(ORDLidar *lidar, char *port, int baudrate)
{
  if (lidar == nullptr || lidar->lidar == nullptr || port == nullptr) { return false; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    drv->setSerialPort(port, baudrate);
    return true;
  }

  return false;
}

bool oradar_connect(ORDLidar *lidar)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) { return drv->connect(); }

  return false;
}
bool oradar_disconnect(ORDLidar *lidar)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) { drv->disconnect(); }

  return true;
}

bool oradar_get_timestamp(ORDLidar *lidar, uint16_t *timestamp)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    *timestamp = drv->getTimestamp();
    return true;
  }

  return false;
}

bool oradar_get_rotation_speed(ORDLidar *lidar, double *rotation_speed)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    *rotation_speed = drv->getRotationSpeed();
    return true;
  }

  return false;
}

bool oradar_get_firmware_version(ORDLidar *lidar, char *top_fw_version, char *bot_fw_version)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  std::string top_fw, bot_fw;
  // str.copy()
  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    if (drv->getFirmwareVersion(top_fw, bot_fw)) {
      strcpy(top_fw_version, top_fw.c_str());
      strcpy(bot_fw_version, bot_fw.c_str());
      return true;
    }
  }

  return false;
}

bool oradar_get_device_sn(ORDLidar *lidar, char *device_sn)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  std::string sn;
  // str.copy()
  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    if (drv->getDeviceSN(sn)) {
      strcpy(device_sn, sn.c_str());
      return true;
    }
  }

  return false;
}

bool oradar_set_rotation_speed(ORDLidar *lidar, uint16_t speed)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) { return drv->setRotationSpeed(speed); }

  return false;
}

bool oradar_activate(ORDLidar *lidar)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) { return drv->activate(); }

  return false;
}

bool oradar_deactive(ORDLidar *lidar)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) { return drv->deactive(); }

  return false;
}

bool oradar_get_grabonescan_blocking(ORDLidar *lidar, one_scan_data_st *data, int timeout_ms)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  one_scan_data_st scan_data;
  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    if (drv->grabOneScanBlocking(scan_data, timeout_ms)) {
      memcpy(data, &scan_data, sizeof(one_scan_data_st));
      return true;
    }
  }

  return false;
}

bool oradar_get_grabfullscan_blocking(ORDLidar *lidar, full_scan_data_st *data, int timeout_ms)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  full_scan_data_st scan_data;
  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    if (drv->grabFullScanBlocking(scan_data, timeout_ms)) {
      memcpy(data, &scan_data, sizeof(full_scan_data_st));
      return true;
    }
  }

  return false;
}

bool oradar_get_grabonescan(ORDLidar *lidar, one_scan_data_st *data)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  one_scan_data_st scan_data;
  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    if (drv->grabOneScan(scan_data)) {
      memcpy(data, &scan_data, sizeof(one_scan_data_st));
      return true;
    }
  }

  return false;
}

bool oradar_get_grabfullscan(ORDLidar *lidar, full_scan_data_st *data)
{
  if (lidar == nullptr || lidar->lidar == nullptr) { return false; }

  full_scan_data_st scan_data;
  OrdlidarDriver *drv = static_cast<OrdlidarDriver *>(lidar->lidar);
  if (drv) {
    if (drv->grabFullScan(scan_data)) {
      memcpy(data, &scan_data, sizeof(full_scan_data_st));
      return true;
    }
  }

  return false;
}
