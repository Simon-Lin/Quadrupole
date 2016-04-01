#include "Vector3D.h"
#include "Sensor.h"
#include "Interface.h"
#include <iostream>
#include <pthread.h>
#include <ncurses.h>
#include <sys/mman.h>

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

  //RT priority of main thread
  sched_param sp_main;
  sp_main.sched_priority = 98;
  sched_setscheduler(0, SCHED_FIFO, &sp_main);

  //force program to RAM
  mlockall(MCL_CURRENT | MCL_FUTURE);

  //start sersor
  terminate = false;
  pthread_t thread_sensor;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  sched_param sp_sensor;
  sp_sensor.sched_priority = 99;
  pthread_attr_setschedparam(&attr, &sp_sensor);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (pthread_create (&thread_sensor, &attr, &Sensor::start, &sensor)) {
    std::cout << "pthread create failed.\n";
    return 1;
  }

  bcm2835_delay(500);
  
  while (1) {
    bcm2835_delay(100);

    mvprintw (2, 0, "g_dir: (% f, % f, % f)", SEN_DATA.g_direction.x, SEN_DATA.g_direction.y, SEN_DATA.g_direction.z);
    mvprintw (3, 0, "omega: (% f, % f, % f)", SEN_DATA.angular_speed.x, SEN_DATA.angular_speed.y, SEN_DATA.angular_speed.z);
    mvprintw (4, 0, "accel: (% f, % f, % f)", SEN_DATA.acceleration.x, SEN_DATA.acceleration.y, SEN_DATA.acceleration.z);
    mvprintw (5, 0, "temp: %fC  pressure: %fpa  batt_v: %fv", SEN_DATA.temperature, SEN_DATA.pressure, SEN_DATA.batt_voltage);
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

