// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#include "FullScanFilter.h"
#include "math/angles.h"
#include <cmath>
#include <string>
#include <vector>

void FullScanFilter::swap(uint16_t *a, uint16_t *b)
{
  uint16_t tmp = *a;
  *a = *b;
  *b = tmp;
}

void FullScanFilter::BubbleSort(uint16_t *data, int len)
{
  for (int i = 0; i < len - 1; ++i) {
    for (int j = 0; j < len - 1 - i; ++j) {
      if (data[j] > data[j + 1]) { swap(&data[j], &data[j + 1]); }
    }
  }
}

bool FullScanFilter::isValidRange(FilterPara ParaInf, uint16_t current_data)
{
  if (current_data >= ParaInf.min_range && current_data <= ParaInf.max_range) { return true; }

  return false;
}


void FullScanFilter::filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out)
{

  //printf("filter_type:%d, %d,%d,%d,%d,%d,%d,%d,%d\n",ParaInf.filter_type, ParaInf.max_range,ParaInf.min_range,\
//ParaInf.sigma_d,ParaInf.sigma_r,ParaInf.intensity_filter_range,ParaInf.weak_intensity_th,ParaInf.rotation,ParaInf.level);
  if (FS_Smooth == ParaInf.filter_type) {
    smooth_filter(in, ParaInf, out);
  } else if (FS_Bilateral == ParaInf.filter_type) {
    bilateral_filter(in, ParaInf, out);
  } else if (FS_Tail == ParaInf.filter_type) {
    tail_filter(in, ParaInf, out);
  } else if (FS_Intensity == ParaInf.filter_type) {
    intensity_filter(in, ParaInf, out);
  }
}


void FullScanFilter::smooth_filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out)
{
  out = in;

  int count = in.vailtidy_point_num;// 一圈的点数
  int window_size = 5;
  if (count < window_size) { return; }

  uint16_t dis_threshold = 40;
  uint16_t data_fifo[5] = { 0 };
  int sum_value = 0;
  int cnt = 0;

  for (int i = (window_size / 2); i < (count - (window_size / 2)); i++) {
    if (isValidRange(ParaInf, in.data[i].distance)) {
      for (int n = 0; n < window_size; n++) { data_fifo[n] = in.data[i - 2].distance; }
      BubbleSort(data_fifo, 5);
      sum_value = 0.0;
      cnt = 0;
      for (int j = 0; j < window_size; j++) {
        if (in.data[i + j - 2].distance == 0) continue;

        if (abs(in.data[i].distance - in.data[i + j - 2].distance) < dis_threshold) {
          cnt++;
          sum_value += in.data[i + j - 2].distance;
        }
      }

      if (cnt != 0) out.data[i].distance = sum_value / cnt;
    }
  }
}

void FullScanFilter::bilateral_filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out)
{

  out = in;

  int circle_len = in.vailtidy_point_num;
  if (circle_len < FILTER_WINDOW_SIZE) { return; }

  int i;
  int j;
  float w_spatial[w_r * 2 + 1], w_distance[w_r * 2 + 1], w_s[w_r * 2 + 1] = { 0 };
  float w_d_sum = 0, w_s_sum = 0;
  uint16_t wins_data[w_r * 2 + 1];/// 窗口数据定义
  int rx[w_r * 2 + 1] = { -1 };
  uint16_t Diff_R = 0, current_data = 0;
  std::vector<uint16_t> temp_dists_data;

  temp_dists_data.resize(circle_len + w_r * 2);

  for (i = 0, j = w_r; i < w_r; i++, j--) {
    temp_dists_data[i] = in.data[circle_len - j].distance;
    temp_dists_data[circle_len + i] = in.data[i].distance;
  }

  // temp_dists_data[0] = in.data[circle_len-2].distance;
  // temp_dists_data[1] = in.data[circle_len-1].distance;
  // temp_dists_data[circle_len] = in.data[0].distance;
  // temp_dists_data[circle_len+1] = in.data[1].distance;
  for (int i = 0; i < circle_len; i++) { temp_dists_data[i + 2] = in.data[i].distance; }

  // 空域核
  for (i = 0; i < w_r * 2 + 1; i++) {
    rx[i] = i - w_r;
    w_spatial[i] = (float)exp(-1 * sqrt(rx[i] * rx[i]) / (2 * ParaInf.sigma_d * ParaInf.sigma_d));
  }

  // 值域核
  for (i = w_r; i < circle_len + 4 - w_r; i++)/// 从第三个点开始，未考虑首尾
  {
    current_data = temp_dists_data[i];/// 待滤波数据

    if (isValidRange(ParaInf, current_data)) {
      for (j = 0; j < w_r * 2 + 1; j++)/// 滑窗内数据
      {
        wins_data[j] = temp_dists_data[i + j - w_r];

        if (wins_data[j] != 0)/// 非零数据有效
        {
          Diff_R = current_data - wins_data[j];
          // w_distance[j] = (float)exp(-1 * (0.003 * ParaInf.sigma_d * (Diff_R * Diff_R)) / (2 * ParaInf.sigma_r *
          // ParaInf.sigma_r));
          w_distance[j] = (float)exp(-1 * ((Diff_R * Diff_R)) / (2 * ParaInf.sigma_r * ParaInf.sigma_r));

          w_s[j] = w_spatial[j] * w_distance[j];

          w_d_sum += wins_data[j] * w_s[j];
          w_s_sum += w_s[j];
        }
      }
      out.data[i - w_r].distance = w_d_sum / w_s_sum;
      w_d_sum = 0.0;
      w_s_sum = 0.0;
    }
  }
}

void FullScanFilter::tail_filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out)
{

  out = in;
  int count = in.vailtidy_point_num;
  if (count == 0) return;

  uint16_t diff_1, diff_2;
  uint16_t LeftPointDistanceCmp = 0, RightPointDistanceCmp = 0;

  double AngleResSin = 0.0;// sin(0.8);
  double AngleResCos = 0.0;// cos(0.8);
  double TanAngleThreshold = 0.0;

  int SumAbnormal = 0;
  int noise_abnormal = 0;
  int PointNumThreshold = 0;
  // int filter_num_test = 0;

  // 转速
  switch (ParaInf.rotation) {

  case 5:
    AngleResSin = 0.006981260297962;// sin(0.4);
    AngleResCos = 0.999975630705395;// cos(0.4);
    break;
  case 6:
    AngleResSin = 0.008377482414771;// sin(0.48);
    AngleResCos = 0.999964908278481;// cos(0.48);
    break;
  case 7:
    AngleResSin = 0.009773688199249;// sin(0.56);
    AngleResCos = 0.999952236368810;// cos(0.56);
    break;
  case 8:
    AngleResSin = 0.011169874929422;// sin(0.64);
    AngleResCos = 0.999937615001086;// cos(0.64);
    break;
  case 9:
    AngleResSin = 0.012566039883353;// sin(0.72);
    AngleResCos = 0.999921044203816;// cos(0.72);
    break;
  case 10:
    AngleResSin = 0.013962180339145;// sin(0.8);
    AngleResCos = 0.999902524009304;// cos(0.8);
    break;
  case 11:
    AngleResSin = 0.015358293574953;// sin(0.88);
    AngleResCos = 0.999882054453657;// cos(0.88);
    break;
  case 12:
    AngleResSin = 0.016754376868981;// sin(0.96);
    AngleResCos = 0.999859635576780;// cos(0.96);
    break;
  case 13:
    AngleResSin = 0.018150427499495;// sin(1.04);
    AngleResCos = 0.999835267422382;// cos(1.04);
    break;
  case 14:
    AngleResSin = 0.019546442744821;// sin(1.12);
    AngleResCos = 0.999808950037968;// cos(1.12);
    break;
  case 15:
    AngleResSin = 0.020942419883357;// sin(1.2);
    AngleResCos = 0.999780683474845;// cos(1.2);
    break;
  }


  switch (ParaInf.level) {
  case 0:
    TanAngleThreshold = 0.140540834702391;
    PointNumThreshold = 2;
    break;
  case 1:
    TanAngleThreshold = 0.267949192431123;
    PointNumThreshold = 2;
    break;
  case 2:
    TanAngleThreshold = 0.324919696232906;
    PointNumThreshold = 2;
    break;
  case 3:
    TanAngleThreshold = 0.363970234266202;
    PointNumThreshold = 1;
    break;
  }


  for (int i = 0; i < count; i++) {
    uint16_t range = in.data[i].distance;// current lidar distance
    double angle = in.data[i].angle;// current lidar angle

    if (range != 0) {
      for (int j = -w_r; j < 0; j++) {
        diff_1 = abs(in.data[i + j].distance - in.data[i].distance);
        diff_2 = abs(in.data[i - j].distance - in.data[i].distance);

        LeftPointDistanceCmp = in.data[i + j].distance;
        RightPointDistanceCmp = in.data[i - j].distance;

        if (diff_1 > 50 && diff_2 > 50) {
          // noise_abnormal++;     //暂时未使用
        }

        double TanAngle = (range * AngleResSin) / (LeftPointDistanceCmp - range * AngleResCos);
        if (diff_1 > 20 && diff_1 < 80) {

          if ((TanAngle < TanAngleThreshold) && (TanAngle > ((-1) * TanAngleThreshold))) { SumAbnormal++; }
        }

        TanAngle = (range * AngleResSin) / (RightPointDistanceCmp - range * AngleResCos);
        if (diff_2 > 20 && diff_2 < 80) {
          if ((TanAngle < TanAngleThreshold) && (TanAngle > ((-1) * TanAngleThreshold))) { SumAbnormal++; }
        }
      }

      if (SumAbnormal >= PointNumThreshold) {
        out.data[i].distance = 0;
        // filter_num_test++;
      }
      // 过滤飞点
      if (noise_abnormal != 0) { out.data[i].distance = 0; }
    }

    SumAbnormal = 0;
    noise_abnormal = 0;
  }
  // printf("filter_num_test:%d\n", filter_num_test);
}


void FullScanFilter::intensity_filter(const full_scan_data_st &in, FilterPara ParaInf, full_scan_data_st &out)
{
  out = in;
  // int filter_num_test = 0;
  int circle_len = in.vailtidy_point_num;
  if (circle_len == 0) { return; }

  for (int i = 0; i < circle_len; i++) {
    if (in.data[i].distance < ParaInf.intensity_filter_range) {
      if (in.data[i].intensity < -ParaInf.weak_intensity_th) {
        out.data[i].distance = 0;
        // filter_num_test++;
      }
    }
  }
  // printf("filter_num_test:%d\n", filter_num_test);
}