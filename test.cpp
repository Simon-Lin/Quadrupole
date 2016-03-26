#include "Vector3D.h"
#include "Sensor.h"
#include "Interface.h"
#include <iostream>
#include <omp.h>
#include <ncurses.h>

int main() {
  SensorData SEN_DATA;
  Sensor sensor(&SEN_DATA, 50);
  sensor.initialize();
  sensor.gyroCalibrate();
  return 0;
  bcm2835_delay(500);
  //  InterfaceData UI_DATA;
  //  Interface UI(&UI_DATA);
  //  UI.initialize();

  initscr();
  cbreak();
  noecho();
  
  bool terminate  = false;
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      sensor.start(terminate);
    }
    #pragma omp section
    {
      while (getch()) {
	bcm2835_delay(100);
	/*	UI_DATA.g_direction = SEN_DATA.g_direction;
	UI_DATA.acceleration = SEN_DATA.acceleration;
	UI_DATA.speed = SEN_DATA.speed;
	UI_DATA.angular_speed = SEN_DATA.angular_speed;
	UI.update();*/
      }
      terminate = true;
    }
  }
  return 0;
}

