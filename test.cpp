#include "Vector3D.h"
#include "Sensor.h"
#include "Interface.h"
#include <iostream>
#include <pthread.h>
#include <ncurses.h>

pthread_spinlock_t I2C_ACCESS;
bool terminate;

int main() {
  SensorData SEN_DATA;
  Sensor sensor(&SEN_DATA, 50);
  sensor.initialize();
  //  sensor.gyroCalibrate();
  bcm2835_delay(500);
  //  InterfaceData UI_DATA;
  //  Interface UI(&UI_DATA);
  //  UI.initialize();

  initscr();
  cbreak();
  noecho();

  //spinlock initialization
  if (pthread_spin_init(&I2C_ACCESS, 0)) {
    std::cout << "pthread spinlock initialization failed.\n";
   return 1;
  }

  //start sersor
  terminate = false;
  pthread_t thread_sensor;
  if (pthread_create (&thread_sensor, NULL, &Sensor::start, &sensor)) {
    std::cout << "pthread create failed.\n";
    return 1;
  }

  while (1) {
    bcm2835_delay(100);

    mvprintw (2, 0, "g_dir: (% f, % f, % f)", SEN_DATA.g_direction.x, SEN_DATA.g_direction.y, SEN_DATA.g_direction.z);
    mvprintw (3, 0, "omega: (% f, % f, % f)", SEN_DATA.angular_speed.x, SEN_DATA.angular_speed.y, SEN_DATA.angular_speed.z);
    mvprintw (5, 0, "temp: %fC  pressure: %fpa", SEN_DATA.temperature, SEN_DATA.pressure);
    refresh();
    
    /*	UI_DATA.g_direction = SEN_DATA.g_direction;
	UI_DATA.acceleration = SEN_DATA.acceleration;
	UI_DATA.speed = SEN_DATA.speed;
	UI_DATA.angular_speed = SEN_DATA.angular_speed;
	UI.update();*/
  }
  terminate = true;

  pthread_join (thread_sensor, NULL);
  pthread_spin_destroy (&I2C_ACCESS);
  
  return 0;
}

