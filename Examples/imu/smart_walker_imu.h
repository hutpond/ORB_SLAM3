/**
 * @file smart_walker_imu.h
 * @author your name (you@domain.com)
 * @brief  深圳智游者IMU数据读写
 * @version 0.1
 * @date 2022-02-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <memory>
#include <thread>
#include <mutex>
#include <vector>

#include "ImuTypes.h"

struct imu_data
{
  double x_acceleration;
  double y_acceleration;
  double z_acceleration;

  double yaw_angular_v;
  double pitch_angular_v;
  double roll_angular_v;

  double yaw;
  double pitch;
  double roll;

  double time_stamps;
};

class smart_walker_imu
{
private:
  int fd_;
  bool flag_rw_;
  std::shared_ptr<std::thread> thread_read_;
  std::mutex mutex_rw_;
  std::vector<std::shared_ptr<imu_data>> imu_datas_;
public:
  smart_walker_imu();
  ~smart_walker_imu();
  void get_data(std::vector<ORB_SLAM3::IMU::Point> &, double time_stamp);
  void stop();
protected:
  void read_data();
};

