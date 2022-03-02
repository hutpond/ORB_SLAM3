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

struct imu_data
{
  double x_acceleration;
  double y_acceleration;
  double z_acceleration;

  double yaw_angular_v;
  double yaw;
  double pitch;
  double roll;
};

class smart_walker_imu
{
private:
  int fd_;
  bool flag_rw_;
  std::shared_ptr<std::thread> thread_read_;
  imu_data imu_data_;
public:
  smart_walker_imu();
  ~smart_walker_imu();
  void stop();
protected:
  void read_data();
};

