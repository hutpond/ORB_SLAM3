#include "smart_walker_imu.h"

#include <iostream>

int main()
{
  smart_walker_imu imu;

  char ch;
  while (true) {
    ch = std::cin.get();
    if (ch == 'q') {
      break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
  imu.stop();

  return 0;
}