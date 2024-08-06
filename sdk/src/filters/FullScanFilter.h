// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifndef FULLSCANFILTER_H
#define FULLSCANFILTER_H
#include "ordlidar_protocol.h"

#define Rth 30
#define Ath 0.8
#define pai 3.1415926
#define DEC (pai / 180)
#define Rad (180 / pai)
#define w_r 2


typedef struct FilterParas
{
  int filter_type = 0;

  int maxRange = 150;
  int minRange = 0;


  int Sigma_D = 5;
  int Sigma_R = 3;

  int IntesntiyFilterRange = 70;/// 距离
  int Weak_Intensity_Th = 31;// 弱信号强度

  // 拖尾滤波
  int Rotation = 10;// 转速，默认,10hz
  int level = 0;// 滤波强度，默认，8°2个点
} FilterPara;


class FullScanFilter
{
public:
  enum FilterStrategy {
    FS_Smooth,
    FS_Bilateral,
    FS_Tail,
    FS_Intensity,
  };

public:
  void filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out);

protected:
  void smooth_filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out);

  void bilateral_filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out);

  void tail_filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out);

  void intensity_filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out);


  bool isValidRange(FilterPara ParaInf, uint16_t current_data);

  void swap(uint16_t *a, uint16_t *b);
  void BubbleSort(uint16_t *data, int len);

protected:
  static const int FILTER_WINDOW_SIZE = 3;// 5;
};

#endif// FULLSCANFILTER_H
