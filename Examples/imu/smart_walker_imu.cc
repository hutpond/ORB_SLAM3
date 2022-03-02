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
  std::cout << "x acc: " << data.x_acceleration << ", "
            << "y acc: " << data.y_acceleration << ", "
            << "z acc: " << data.z_acceleration << ", "
            << "yaw ang v: " << data.yaw_angular_v << ", "
            << "yaw: " << data.yaw << ", "
            << "pitch: " << data.pitch << ", "
            << "roll: " << data.roll << std::endl;
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

void smart_walker_imu::read_data()
{
  constexpr int Data_Len = 16;
  const double range = std::pow(2, 15);
  const double gravity = 9.794;

  uint8_t data[1024];
  uint8_t crc_data[2];
  int16_t raw_data;

  flag_rw_ = true;
  uint32_t pnt_idx = 0;
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

    // x acceleration
    size_t index = 2;
    memcpy(&raw_data, data + index, 2);
    imu_data_.x_acceleration = raw_data * gravity * 2 / range;

    // y acceleration
    index += 2;
    memcpy(&raw_data, data + index, 2);
    imu_data_.y_acceleration = raw_data * gravity * 2 / range;

    // z acceleration
    index += 2;
    memcpy(&raw_data, data + index, 2);
    imu_data_.z_acceleration = raw_data * gravity * 2 / range;

    // yaw angular velocity
    index += 2;
    memcpy(&raw_data, data + index, 2);
    imu_data_.yaw_angular_v = raw_data / 100.0;

    // yaw
    index += 2;
    memcpy(&raw_data, data + index, 2);
    imu_data_.yaw = raw_data / 100.0;

    // pitch
    index += 2;
    memcpy(&raw_data, data + index, 2);
    imu_data_.pitch = raw_data / 100.0;

    // roll
    index += 2;
    memcpy(&raw_data, data + index, 2);
    imu_data_.roll = raw_data / 100.0;

    if (++ pnt_idx % 150 == 0) {
      std::cout << imu_data_;
    }
  }
}
