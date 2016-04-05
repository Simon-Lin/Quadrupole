#include "Sensor.h"
#include "Interface.h"
#include "Controller.h"
#include "Vector3D.h"
#include <bcm2835.h>
#include <iostream>
#include <pthread.h>
#include <sys/mman.h>

pthread_spinlock_t I2C_ACCESS;
bool terminate;

bool controlcycle (Sensor &sensor, Interface &interface, Controller &controller, SensorData &SEN_DATA, InterfaceData &INT_DATA) {
  bcm2835_delay (100);
  
  INT_DATA.g_direction = SEN_DATA.g_direction;
  INT_DATA.acceleration = SEN_DATA.acceleration;
  INT_DATA.speed = SEN_DATA.speed;
  INT_DATA.angular_speed = SEN_DATA.angular_speed;
  INT_DATA.pressure = SEN_DATA.pressure;
  INT_DATA.temperature = SEN_DATA.temperature;
  interface.update();
    
  if (INT_DATA.att_hold) {
    controller.control_HoldAtt (SEN_DATA.speed.z, SEN_DATA.angular_speed.z, INT_DATA.yaw_set, SEN_DATA.g_direction, INT_DATA.g_direction_set);
  } else {
    controller.control (INT_DATA.throttle, SEN_DATA.angular_speed.z, INT_DATA.yaw_set, SEN_DATA.g_direction, INT_DATA.g_direction_set);  
  }

  if (INT_DATA.power_off) {
    terminate = true;
    return false;
  } else {
    return true;
  }
}

int main (int argc, char *argv[]) {
  system("clear");
  std::cout << "Quadrupole controller side ver 0.1\n";
  std::cout << "Initializing sensors...\n";
  SensorData SEN_DATA;
  Sensor sensor(&SEN_DATA, 50);
  if (!sensor.initialize()) return 1;

  std::cout << "Intiailzing network server...\n";
  InterfaceData INT_DATA;
  Interface interface(&INT_DATA);
  if (!interface.initialize()) return 1;

  std::cout << "Initializing servo controller...\n";
  ControlParameters PARA;
  PARA.bal_lin = -1;
  PARA.bal_diff = PARA.bal_int = 0.0001;
  PARA.yaw_lin = -1;
  PARA.yaw_diff = 0;
  PARA.att_con = PARA.att_lin = PARA.att_diff = PARA.att_int = 0.0001;
  Controller controller (PARA);
  if (!controller.initialize()) return 1;

  //setting scheduling policy of main thread
  sched_param sp_main;
  sp_main.sched_priority = 98;
  sched_setscheduler(0, SCHED_FIFO, &sp_main);

  //force program to RAM
  mlockall(MCL_CURRENT | MCL_FUTURE);
  
  //spinlock initialization
  if (pthread_spin_init(&I2C_ACCESS, 0)) {
    std::cout << "pthread spinlock initialization failed.\n";
   return 1;
  }
  
  //start collecting sensor data
  terminate = false;
  pthread_t thread_sensor;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  sched_param sp_sensor;
  sp_sensor.sched_priority = 99;
  pthread_attr_setschedparam(&attr, &sp_sensor);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (pthread_create (&thread_sensor, NULL, &Sensor::start, &sensor)) {
    std::cout << "pthread create failed.\n";
    return 1;
  }

  bcm2835_delay(500);
  std::cout << "Startup process complete! Ready for a flight.\n";
  
  //main loop
  while (controlcycle(sensor, interface, controller, SEN_DATA, INT_DATA));

  //destructors
  pthread_join (thread_sensor, NULL);
  pthread_spin_destroy (&I2C_ACCESS);
  
  std::cout << "bye!\n";
  return 0;
}
