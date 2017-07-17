#include "Sensor.h"
#include "Interface.h"
#include "Controller.h"
#include <iostream>
#include <pthread.h>
#include <ncurses.h>
#include <sys/mman.h>

int main() {
  Data DATA;

  //spinlock initialization
  if (pthread_spin_init(&(DATA.I2C_ACCESS), 0)) {
    std::cout << "pthread spinlock initialization failed.\n";
   return 1;
  }

  Sensor sensor(&DATA, 10);
  sensor.initialize();
  sensor.gyroCalibrate();
  sensor.accelCalibrate();
  sensor.IMU_GetOffsets();
  sensor.IMU_SelfTest();
  return 0;
  bcm2835_delay(500);
  Controller controller (&DATA);
  controller.initialize();
  //  InterfaceData UI_DATA;
  //  Interface UI(&UI_DATA);
  //  UI.initialize();

  initscr();
  cbreak();
  noecho();
  nodelay(stdscr, TRUE);

  //RT priority of main thread
  sched_param sp_main;
  sp_main.sched_priority = 98;
  sched_setscheduler(0, SCHED_FIFO, &sp_main);

  //force program to RAM
  mlockall(MCL_CURRENT | MCL_FUTURE);

  //start sersor
  DATA.terminate = false;
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

  ServoData XD;
  double power = 0;
  bcm2835_delay(500);
  
  while (getch() == ERR) {
    bcm2835_delay(100);

    if (power < 1) {
      power += 0.01;
    } else {
      power = 0;
    }

    XD.UL = XD.DL = XD.DR = XD.UR = power;
    mvprintw (7, 0, "Servo: % f, % f, % f, % f", XD.UL, XD.UR, XD.DL, XD.DR);
    controller.setServo(XD);
    
    mvprintw (2, 0, "g_dir: (% f, % f, % f)", DATA.g_direction[0], DATA.g_direction[1], DATA.g_direction[2]);
    mvprintw (3, 0, "omega: (% f, % f, % f)", DATA.angular_speed[0], DATA.angular_speed[1], DATA.angular_speed[2]);
    mvprintw (4, 0, "accel: (% f, % f, % f)", DATA.acceleration[0], DATA.acceleration[1], DATA.acceleration[2]);
    mvprintw (5, 0, "temp: %fC  pressure: %fpa  batt_v: %fv", DATA.temperature, DATA.pressure, DATA.batt_voltage);
    refresh();
    
    /*	UI_DATA.g_direction = DATA.g_direction;
	UI_DATA.acceleration = DATA.acceleration;
	UI_DATA.speed = DATA.speed;
	UI_DATA.angular_speed = DATA.angular_speed;
	UI.update();*/
  }
  DATA.terminate = true;

  pthread_join (thread_sensor, NULL);
  pthread_spin_destroy (&(DATA.I2C_ACCESS));

  endwin();
  return 0;
}
