/**
 * @file smart_walker_imu.cc
 * @author your name (you@domain.com)
 * @brief  深圳智游者IMU数据读写
 * @version 0.1
 * @date 2022-02-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "smart_walker_imu.h"
#include "serial.hpp"
// #include "linux_serial.hpp"
// #include "linux_serial_2.hpp"

#include <cstring>
#include <cmath>
#include <iostream>

std::ostream & operator << (std::ostream &out, const imu_data &data) {
  std::cout << "acceration: " << data.x_acceleration << ", "
            << data.y_acceleration << ", "
            << data.z_acceleration << std::endl
            << "ang vel: " << data.yaw_angular_v << ", "
            << data.pitch_angular_v << ", "
            << data.roll_angular_v << std::endl
            << "angle: " << data.yaw << ", "
            << data.pitch << ", "
            << data.roll << std::endl << std::endl;
  return out;
}

  double x_acceleration;
  double y_acceleration;
  double z_acceleration;

  double yaw_angular_v;
  double yaw;
  double pitch;
  double roll;


smart_walker_imu::smart_walker_imu()
{
  serial::setup_serial_port("/dev/ttyUSB0", B115200, 8, 1, 'N', 0);
  fd_ = serial::open_device("/dev/ttyUSB0");
  serial::set_speed(fd_, 115200);
  serial::set_parity(fd_, 8, 1, 'N');
  serial::set_flow_ctrl(fd_, 0);
  serial::set_common_props(fd_);

  thread_read_.reset(new std::thread(&smart_walker_imu::read_data, this));
}

smart_walker_imu::~smart_walker_imu()
{
}

void smart_walker_imu::stop()
{
  flag_rw_ = false;
  thread_read_->join();
}

void smart_walker_imu::get_data(std::vector<ORB_SLAM3::IMU::Point> &imu_meas, double time_stamp)
{
  std::lock_guard<std::mutex> lock(mutex_rw_);
  const int Size = imu_datas_.size();
  for (int i = 0; i < Size; ++i) {
    // if (imu_datas_[i]->time_stamps <= time_stamp) {
      imu_meas.push_back(ORB_SLAM3::IMU::Point(
        imu_datas_[i]->x_acceleration, imu_datas_[i]->y_acceleration, imu_datas_[i]->z_acceleration,
        imu_datas_[i]->yaw_angular_v, imu_datas_[i]->pitch_angular_v, imu_datas_[i]->roll_angular_v,
        imu_datas_[i]->time_stamps));
    // }
  }
  imu_datas_.clear();
}

void smart_walker_imu::read_data()
{
  constexpr int Data_Len = 16;
  const double range = std::pow(2, 15);
  const double gravity = 9.794;

  uint8_t data[1024];
  uint8_t crc_data[2];

  flag_rw_ = true;
  uint32_t pnt_idx = 0;
  auto s_t1 = std::chrono::duration_cast<std::chrono::nanoseconds>
    (std::chrono::high_resolution_clock::now().time_since_epoch()).count() / (1000.0 * 1000.0 * 1000.0);
  double pitch_old = 0;
  double roll_old = 0;
  
  while(flag_rw_) {
    size_t len = read(fd_, data, 1);
    if (len != 1 || data[0] != 0xA5) {
      std::cout << "flag 0 error " << len << " " << (int)data[0] << std::endl;
      continue;
    }
    len = read(fd_, data + 1, 1);
    if (len != 1 || data[1] != 0xA5) {
      std::cout << "flag 1 error " << len << " " << (int)data[1] << std::endl;
      continue;
    }

    int recv_len = read(fd_, data + 2, Data_Len);
    if (recv_len != Data_Len) {
      std::cout << "data len error" << std::endl;
      continue;
    }

    // sum check
    uint16_t sum_check = 0;
    size_t sum_idx = 8;
    for (int i = 0; i < 4; ++i) {
      uint16_t sum_item = 0;
      memcpy(&sum_item, data + sum_idx + 2 * i, 2);
      sum_check += sum_item;
    }
    memcpy(crc_data, &sum_check, 2);
    if (crc_data[0] != data[Data_Len] || crc_data[1] != data[Data_Len + 1]) {
      std::cout << "crc error" << (int)crc_data[0] << " " << (int)crc_data[1] << " " << 
                   (int)data[Data_Len] << " " << (int)data[Data_Len + 1] << " " << std::endl;
      continue;
    }

    std::shared_ptr<imu_data> cur_imu_data = std::make_shared<imu_data>();
    cur_imu_data->time_stamps = (double)std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::high_resolution_clock::now().time_since_epoch()).count() / (1000 * 1000 * 1000);
    auto s_t2 = cur_imu_data->time_stamps;

    int16_t raw_data;
    // int16_t raw_data_yaw_angv;
    // int16_t raw_data_yaw, raw_data_pitch, raw_data_roll;

    // x acceleration
    size_t index = 2;
    memcpy(&raw_data, data + index, 2);
    cur_imu_data->z_acceleration = raw_data * gravity / 1024.0;
    // y acceleration
    index += 2;
    memcpy(&raw_data, data + index, 2);
    cur_imu_data->y_acceleration = raw_data * gravity / 1024.0;
    // z acceleration
    index += 2;
    memcpy(&raw_data, data + index, 2);
    cur_imu_data->x_acceleration = raw_data * gravity / 1024.0;
    // yaw angular velocity
    index += 2;
    memcpy(&raw_data, data + index, 2);
    cur_imu_data->yaw_angular_v = raw_data / 100.0 / 180.0 * M_PI;
    // yaw
    index += 2;
    memcpy(&raw_data, data + index, 2);
    cur_imu_data->yaw = raw_data / 100.0;
    // pitch
    index += 2;
    memcpy(&raw_data, data + index, 2);
    cur_imu_data->pitch = raw_data / 100.0;
    // roll
    index += 2;
    memcpy(&raw_data, data + index, 2);
    cur_imu_data->roll = raw_data / 100.0;

    cur_imu_data->pitch_angular_v = (cur_imu_data->pitch - pitch_old) / (s_t2 - s_t1) / 180.0 * M_PI;
    cur_imu_data->roll_angular_v = (cur_imu_data->roll - roll_old) / (s_t2 - s_t1) / 180.0 * M_PI;

    pitch_old = cur_imu_data->pitch;
    roll_old = cur_imu_data->roll;
    s_t1 = s_t2;

    // data
    {
      std::lock_guard<std::mutex> lock(mutex_rw_);
      imu_datas_.push_back(cur_imu_data);
    }

    // if (++ pnt_idx % 150 == 0) {
    //   std::cout << *cur_imu_data;
    // }
  }
}
