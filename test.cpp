#include "Vector3D.h"
#include "Sensor.h"
#include <sys/select.h>
#include <iostream>
#include <omp.h>

int kbhit();
void printdata(SensorData &DATA, bool &terminate);

int main() {
  SensorData DATA;
  Sensor sensor(&DATA, 50);
  sensor.initialize();
  bool terminate = false;
  //  sensor.IMU_Calibrate();
  //  sensor.IMU_SelfTest();

  #pragma omp parallel sections
  {
    #pragma omp section
    {
      std::cout << "thread of printdata: "  << omp_get_thread_num() << std::endl << std::flush;
      printdata(DATA, terminate);
    }
    #pragma omp section
    {
      std::cout << "thread of sensor: " <<  omp_get_thread_num() << std::endl << std::flush;
      sensor.start(terminate);
    }
  }
  return 0;
}

void printdata(SensorData &DATA, bool &terminate) {
  while (!kbhit()) {
    std::cout << "\naccel: ";
    DATA.acceleration.print();
    std::cout << "   omega: ";
    DATA.angular_speed.print();
    std::cout << "   g: ";
    DATA.g_direction.print();
    
    bcm2835_delay(100);
  }
  terminate = true;
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
