#include "Vector3D.h"
#include "Sensor.h"
#include "Interface.h"
#include <sys/select.h>
#include <iostream>
#include <omp.h>

int kbhit();

int main() {
  SensorData SEN_DATA;
  Sensor sensor(&SEN_DATA, 50);
  sensor.initialize();

  bcm2835_delay(500);
  InterfaceData UI_DATA;
  Interface UI(&UI_DATA);
  UI.initialize();
  
  bool terminate = false;
  //  sensor.IMU_Calibrate();
  //  sensor.IMU_SelfTest();

  #pragma omp parallel sections
  {
    #pragma omp section
    {
      sensor.start(terminate);
    }
    #pragma omp section
    {
      while (!kbhit()) {
	bcm2835_delay(100);
	UI_DATA.g_direction = SEN_DATA.g_direction;
	UI_DATA.acceleration = SEN_DATA.g_direction;
	UI_DATA.speed = SEN_DATA.speed;
	UI_DATA.angular_speed = SEN_DATA.angular_speed;
	UI.update();
      }
      terminate = true;
    }
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
