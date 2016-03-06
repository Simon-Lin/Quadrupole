#include "Vector3D.h"
#include "Sensor.h"
#include <sys/select.h>
#include <iostream>

int kbhit();

int main() {
  Sensor sensor(10);
  Vector3D accel, speed, omega, rotation;
  sensor.initialize();

  while (1) {
    if (kbhit()) break;

    sensor.getMotionData(accel, speed, omega, rotation);
    std::cout << "\naccel: ";
    accel.print();
    std::cout << "   speed: ";
    speed.print();
    std::cout << "   omega: ";
    omega.print();
    std::cout << "   rotation: ";
    rotation.print();
  }
  return 0;
}

int kbhit()
{
  struct timeval tv;
  fd_set read_fd;

  tv.tv_sec=0;
  tv.tv_usec=0;
  FD_ZERO(&read_fd);
  FD_SET(0,&read_fd);

  if(select(1, &read_fd, NULL, NULL, &tv) == -1)
    return 0;

  if(FD_ISSET(0,&read_fd))
    return 1;

  return 0;
}
